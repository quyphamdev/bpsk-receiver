-- =============================================================================
-- Module: carrier_recovery_costas.vhd
-- Description: Costas loop for BPSK carrier recovery.
--
--              Phase detector (decision-directed):
--                error = I_in * sign(Q_in)
--              (For BPSK, the Q component should ideally be zero; any residual
--               Q represents phase error.)
--
--              Loop filter: PI (proportional + integral) controller
--                loop_out[n] = Kp * error[n] + Ki * sum(error)
--
--              The loop filter output drives a phase NCO.  The NCO applies a
--              complex rotation to the incoming samples to remove residual
--              carrier phase/frequency error.
--
--              A lock detector monitors the magnitude of the loop error and
--              asserts "locked" when the error is below a threshold for a
--              configurable number of consecutive cycles.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity carrier_recovery_costas is
    generic (
        DATA_WIDTH   : positive := 16;    -- I/Q sample width
        PHASE_WIDTH  : positive := 24;    -- NCO phase accumulator width
        LUT_DEPTH    : positive := 256;   -- Quarter-wave LUT size
        -- Loop filter gains (fixed-point, scaled by 2^GAIN_SHIFT)
        KP           : integer  := 128;   -- Proportional gain
        KI           : integer  := 4;     -- Integral gain
        GAIN_SHIFT   : positive := 12;    -- Right-shift applied after multiply
        -- Lock detector
        LOCK_THRESH  : positive := 64;    -- Error magnitude threshold for lock
        LOCK_COUNT   : positive := 1024   -- Cycles below threshold to declare lock
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- Input I/Q samples (from NCO frequency corrector)
        in_valid    : in  std_logic;
        in_i        : in  signed(DATA_WIDTH - 1 downto 0);
        in_q        : in  signed(DATA_WIDTH - 1 downto 0);

        -- Phase-corrected output
        out_valid   : out std_logic;
        out_i       : out signed(DATA_WIDTH - 1 downto 0);
        out_q       : out signed(DATA_WIDTH - 1 downto 0);

        -- Lock indicator
        locked      : out std_logic
    );
end entity carrier_recovery_costas;

architecture rtl of carrier_recovery_costas is

    -- Quarter-wave cosine LUT
    constant LUT_WIDTH : positive := 16;
    constant LUT_SCALE : real     := real(2 ** (LUT_WIDTH - 1)) - 1.0;

    type lut_array_t is array (0 to LUT_DEPTH - 1) of signed(LUT_WIDTH - 1 downto 0);

    impure function gen_cos_lut return lut_array_t is
        variable tbl : lut_array_t;
    begin
        for i in 0 to LUT_DEPTH - 1 loop
            tbl(i) := to_signed(integer(round(
                cos(MATH_PI / 2.0 * real(i) / real(LUT_DEPTH)) * LUT_SCALE)),
                LUT_WIDTH);
        end loop;
        return tbl;
    end function;

    constant COS_LUT : lut_array_t := gen_cos_lut;

    -- Phase accumulator for Costas NCO
    signal phase_acc  : signed(PHASE_WIDTH - 1 downto 0) := (others => '0');

    -- Loop filter integrator
    signal integrator : signed(PHASE_WIDTH - 1 downto 0) := (others => '0');

    -- LUT outputs
    signal cos_val    : signed(LUT_WIDTH - 1 downto 0) := (others => '0');
    signal sin_val    : signed(LUT_WIDTH - 1 downto 0) := (others => '0');

    -- Pipeline stage 1: inputs + LUT
    signal p1_valid   : std_logic := '0';
    signal p1_i       : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal p1_q       : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal p1_cos     : signed(LUT_WIDTH - 1 downto 0) := (others => '0');
    signal p1_sin     : signed(LUT_WIDTH - 1 downto 0) := (others => '0');

    -- Pipeline stage 2: rotation products
    constant PROD_WIDTH : positive := DATA_WIDTH + LUT_WIDTH;
    signal p2_valid   : std_logic := '0';
    signal p2_i_cos   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_q_sin   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_q_cos   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_i_sin   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');

    -- Rotated I/Q (output of rotation stage)
    signal rot_i      : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal rot_q      : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal rot_valid  : std_logic := '0';

    -- Lock detector
    signal lock_cnt   : integer range 0 to LOCK_COUNT := 0;
    signal locked_r   : std_logic := '0';

begin

    -- -------------------------------------------------------------------------
    -- LUT addressing: full-circle from PHASE_WIDTH phase accumulator
    -- -------------------------------------------------------------------------
    proc_lut : process(clk) is
        variable quad : unsigned(1 downto 0);
        variable idx  : integer range 0 to LUT_DEPTH - 1;
        variable ph   : unsigned(PHASE_WIDTH - 1 downto 0);
        variable c, s : signed(LUT_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cos_val <= (others => '0');
                sin_val <= (others => '0');
            else
                ph   := unsigned(phase_acc);
                quad := ph(PHASE_WIDTH - 1 downto PHASE_WIDTH - 2);
                idx  := to_integer(
                    ph(PHASE_WIDTH - 3 downto PHASE_WIDTH - 2 - integer(ceil(log2(real(LUT_DEPTH)))))
                );

                case quad is
                    when "00" =>
                        c :=  COS_LUT(idx);
                        s :=  COS_LUT(LUT_DEPTH - 1 - idx);
                    when "01" =>
                        c := -COS_LUT(LUT_DEPTH - 1 - idx);
                        s :=  COS_LUT(idx);
                    when "10" =>
                        c := -COS_LUT(idx);
                        s := -COS_LUT(LUT_DEPTH - 1 - idx);
                    when others =>
                        c :=  COS_LUT(LUT_DEPTH - 1 - idx);
                        s := -COS_LUT(idx);
                end case;
                cos_val <= c;
                sin_val <= s;
            end if;
        end if;
    end process proc_lut;

    -- -------------------------------------------------------------------------
    -- Pipeline: apply complex rotation I_out = I*cos + Q*sin, Q_out = Q*cos - I*sin
    -- -------------------------------------------------------------------------
    proc_p1 : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                p1_valid <= '0';
            else
                p1_valid <= in_valid;
                if in_valid = '1' then
                    p1_i   <= in_i;
                    p1_q   <= in_q;
                    p1_cos <= cos_val;
                    p1_sin <= sin_val;
                end if;
            end if;
        end if;
    end process proc_p1;

    proc_p2 : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                p2_valid <= '0';
            else
                p2_valid <= p1_valid;
                if p1_valid = '1' then
                    p2_i_cos <= p1_i * p1_cos;
                    p2_q_sin <= p1_q * p1_sin;
                    p2_q_cos <= p1_q * p1_cos;
                    p2_i_sin <= p1_i * p1_sin;
                end if;
            end if;
        end if;
    end process proc_p2;

    proc_rot : process(clk) is
        variable si, sq : signed(PROD_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                rot_valid <= '0';
                rot_i     <= (others => '0');
                rot_q     <= (others => '0');
            else
                rot_valid <= p2_valid;
                if p2_valid = '1' then
                    si := p2_i_cos + p2_q_sin;
                    sq := p2_q_cos - p2_i_sin;
                    -- Remove LUT scale by shifting right LUT_WIDTH-1, then take
                    -- bottom DATA_WIDTH bits of the shifted result.
                    rot_i <= resize(shift_right(si, LUT_WIDTH - 1)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
                    rot_q <= resize(shift_right(sq, LUT_WIDTH - 1)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
                end if;
            end if;
        end if;
    end process proc_rot;

    -- -------------------------------------------------------------------------
    -- Phase detector and loop filter (Costas loop update)
    -- error = rot_i * sign(rot_q)
    -- -------------------------------------------------------------------------
    proc_loop : process(clk) is
        -- Use 9-bit signed for gains so KP up to 255 and KI fit without overflow
        constant KP_SLV   : signed(8 downto 0) := to_signed(KP, 9);
        constant KI_SLV   : signed(8 downto 0) := to_signed(KI, 9);
        variable error      : signed(DATA_WIDTH - 1 downto 0);
        variable prop_term  : signed(DATA_WIDTH + 8 downto 0);
        variable integ_term : signed(PHASE_WIDTH - 1 downto 0);
        variable loop_out   : signed(PHASE_WIDTH - 1 downto 0);
        variable abs_err    : integer;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                phase_acc  <= (others => '0');
                integrator <= (others => '0');
                locked_r   <= '0';
                lock_cnt   <= 0;
            elsif rot_valid = '1' then
                -- Phase detector (BPSK Costas): error = Q * sign(I)
                -- Correct formula: zero at lock (Q=0, I≠0), proportional to sin(phase_error).
                -- The original I*sign(Q) was a bang-bang detector locked to phi=pi/2, not phi=0.
                if rot_i >= 0 then
                    error := rot_q;
                else
                    error := -rot_q;
                end if;

                -- Loop filter: PI
                -- KP_SLV / KI_SLV are 9-bit; error is DATA_WIDTH-bit
                -- products are DATA_WIDTH+9 bits, resized as needed
                prop_term  := resize(KP_SLV * error, DATA_WIDTH + 9);
                integrator <= integrator + resize(KI_SLV * error, PHASE_WIDTH);
                loop_out   := resize(shift_right(prop_term, GAIN_SHIFT), PHASE_WIDTH)
                              + shift_right(integrator, GAIN_SHIFT);

                -- NCO update
                phase_acc <= phase_acc + loop_out;

                -- Lock detector
                abs_err := to_integer(error) when to_integer(error) >= 0
                           else -to_integer(error);
                if abs_err < LOCK_THRESH then
                    if lock_cnt = LOCK_COUNT then
                        locked_r <= '1';
                    else
                        lock_cnt <= lock_cnt + 1;
                    end if;
                else
                    lock_cnt <= 0;
                    locked_r <= '0';
                end if;
            end if;
        end if;
    end process proc_loop;

    out_valid <= rot_valid;
    out_i     <= rot_i;
    out_q     <= rot_q;
    locked    <= locked_r;

end architecture rtl;
