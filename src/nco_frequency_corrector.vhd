-- =============================================================================
-- Module: nco_frequency_corrector.vhd
-- Description: Numerically Controlled Oscillator (NCO) + complex frequency
--              corrector.
--
--              A 32-bit phase accumulator advances by a frequency control word
--              (FCW) each sample.  The sine and cosine of the current phase are
--              read from a quarter-wave lookup table (256 entries per quarter,
--              1024 total equivalent; only 256 are stored, with symmetry logic
--              to reconstruct the full 360 degrees).
--
--              The complex rotation is then applied:
--                I_out = I_in * cos(θ) + Q_in * sin(θ)
--                Q_out = Q_in * cos(θ) - I_in * sin(θ)
--
--              The multipliers are pipelined: 2 register stages.
--
--              Frequency control word (FCW):
--                FCW = round(f_offset / f_s * 2^32)
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity nco_frequency_corrector is
    generic (
        DATA_WIDTH  : positive := 16;    -- I/Q sample width
        PHASE_WIDTH : positive := 32;    -- Phase accumulator width
        LUT_DEPTH   : positive := 256    -- Quarter-wave LUT entries
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- Frequency control word (signed, from freq estimator)
        fcw         : in  signed(PHASE_WIDTH - 1 downto 0);

        -- Input I/Q samples
        in_valid    : in  std_logic;
        in_i        : in  signed(DATA_WIDTH - 1 downto 0);
        in_q        : in  signed(DATA_WIDTH - 1 downto 0);

        -- Output frequency-corrected I/Q samples
        out_valid   : out std_logic;
        out_i       : out signed(DATA_WIDTH - 1 downto 0);
        out_q       : out signed(DATA_WIDTH - 1 downto 0)
    );
end entity nco_frequency_corrector;

architecture rtl of nco_frequency_corrector is

    -- Quarter-wave LUT: stores cos values for phase 0 .. pi/2
    constant LUT_WIDTH : positive := 16;  -- Precision of LUT entries
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

    -- Phase accumulator
    signal phase_acc : unsigned(PHASE_WIDTH - 1 downto 0) := (others => '0');

    -- LUT address and quadrant
    signal lut_addr  : integer range 0 to LUT_DEPTH - 1 := 0;
    signal quadrant  : unsigned(1 downto 0) := "00";

    -- cos and sin from LUT with sign corrections
    signal cos_val   : signed(LUT_WIDTH - 1 downto 0) := (others => '0');
    signal sin_val   : signed(LUT_WIDTH - 1 downto 0) := (others => '0');

    -- Pipeline stage 1: registered inputs
    signal p1_valid  : std_logic := '0';
    signal p1_i      : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal p1_q      : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal p1_cos    : signed(LUT_WIDTH - 1 downto 0) := (others => '0');
    signal p1_sin    : signed(LUT_WIDTH - 1 downto 0) := (others => '0');

    -- Pipeline stage 2: products
    constant PROD_WIDTH : positive := DATA_WIDTH + LUT_WIDTH;
    signal p2_valid   : std_logic := '0';
    signal p2_i_cos   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_q_sin   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_q_cos   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');
    signal p2_i_sin   : signed(PROD_WIDTH - 1 downto 0) := (others => '0');

begin

    -- -------------------------------------------------------------------------
    -- Phase accumulator: advances by FCW each valid input sample
    -- -------------------------------------------------------------------------
    proc_acc : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                phase_acc <= (others => '0');
            elsif in_valid = '1' then
                phase_acc <= phase_acc + unsigned(fcw);
            end if;
        end if;
    end process proc_acc;

    -- -------------------------------------------------------------------------
    -- LUT addressing: map phase to quarter-wave cos/sin
    -- Top 2 bits select quadrant; next log2(LUT_DEPTH) bits select entry
    -- -------------------------------------------------------------------------
    proc_lut : process(clk) is
        variable ph     : unsigned(PHASE_WIDTH - 1 downto 0);
        variable quad   : unsigned(1 downto 0);
        variable idx    : integer range 0 to LUT_DEPTH - 1;
        variable c_val  : signed(LUT_WIDTH - 1 downto 0);
        variable s_val  : signed(LUT_WIDTH - 1 downto 0);
        variable lut_i  : integer range 0 to LUT_DEPTH - 1;
        variable lut_ci : integer range 0 to LUT_DEPTH - 1;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cos_val <= (others => '0');
                sin_val <= (others => '0');
            else
                ph   := phase_acc;
                quad := ph(PHASE_WIDTH - 1 downto PHASE_WIDTH - 2);
                -- Map to quarter-wave index
                idx  := to_integer(
                    ph(PHASE_WIDTH - 3 downto PHASE_WIDTH - 2 - integer(ceil(log2(real(LUT_DEPTH)))))
                );

                case quad is
                    when "00" =>
                        -- Q0: cos = cos(x), sin = sin(x) = cos(pi/2 - x)
                        c_val := COS_LUT(idx);
                        s_val := COS_LUT(LUT_DEPTH - 1 - idx);
                    when "01" =>
                        -- Q1: cos = -sin(x), sin = cos(x)
                        c_val := -COS_LUT(LUT_DEPTH - 1 - idx);
                        s_val :=  COS_LUT(idx);
                    when "10" =>
                        -- Q2: cos = -cos(x), sin = -sin(x)
                        c_val := -COS_LUT(idx);
                        s_val := -COS_LUT(LUT_DEPTH - 1 - idx);
                    when others =>
                        -- Q3: cos = sin(x), sin = -cos(x)
                        c_val :=  COS_LUT(LUT_DEPTH - 1 - idx);
                        s_val := -COS_LUT(idx);
                end case;

                cos_val <= c_val;
                sin_val <= s_val;
            end if;
        end if;
    end process proc_lut;

    -- -------------------------------------------------------------------------
    -- Pipeline stage 1: register I/Q inputs and LUT values together
    -- -------------------------------------------------------------------------
    proc_p1 : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                p1_valid <= '0';
                p1_i     <= (others => '0');
                p1_q     <= (others => '0');
                p1_cos   <= (others => '0');
                p1_sin   <= (others => '0');
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

    -- -------------------------------------------------------------------------
    -- Pipeline stage 2: four multiplications
    --   I*cos, Q*sin, Q*cos, I*sin
    -- -------------------------------------------------------------------------
    proc_p2 : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                p2_valid <= '0';
                p2_i_cos <= (others => '0');
                p2_q_sin <= (others => '0');
                p2_q_cos <= (others => '0');
                p2_i_sin <= (others => '0');
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

    -- -------------------------------------------------------------------------
    -- Pipeline stage 3: additions + truncation back to DATA_WIDTH
    -- I_out = I*cos + Q*sin
    -- Q_out = Q*cos - I*sin
    -- Truncate by LUT_WIDTH-1 bits (remove LUT scale)
    -- -------------------------------------------------------------------------
    proc_p3 : process(clk) is
        variable sum_i : signed(PROD_WIDTH - 1 downto 0);
        variable sum_q : signed(PROD_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                out_valid <= '0';
                out_i     <= (others => '0');
                out_q     <= (others => '0');
            else
                out_valid <= p2_valid;
                if p2_valid = '1' then
                    sum_i := p2_i_cos + p2_q_sin;
                    sum_q := p2_q_cos - p2_i_sin;
                    -- Truncate: shift right by LUT_WIDTH-1 to remove LUT scale,
                    -- then take the bottom DATA_WIDTH bits of the shifted result.
                    out_i <= resize(shift_right(sum_i, LUT_WIDTH - 1)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
                    out_q <= resize(shift_right(sum_q, LUT_WIDTH - 1)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
                end if;
            end if;
        end if;
    end process proc_p3;

end architecture rtl;
