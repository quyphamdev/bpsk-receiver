-- =============================================================================
-- Module: matched_filter.vhd
-- Description: Parameterizable complex FIR matched filter for pulse-shape
--              filtering.  The filter coefficients model a root-raised-cosine
--              (RRC) pulse shape; they can be overridden via the COEFFS
--              generic.  A shift-register + multiply-accumulate (MAC)
--              architecture is used with pipeline registers at the output for
--              timing closure.
--
--              Bit-growth: with NUM_TAPS taps and DATA_WIDTH input, the MAC
--              accumulator grows by log2(NUM_TAPS) bits.  The output is
--              truncated back to DATA_WIDTH after scaling.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity matched_filter is
    generic (
        DATA_WIDTH  : positive := 16;   -- I/Q input sample width (signed)
        COEFF_WIDTH : positive := 16;   -- Filter coefficient width (signed)
        NUM_TAPS    : positive := 16;   -- Number of FIR taps
        -- Default RRC-like coefficients (fixed-point, scaled to COEFF_WIDTH)
        -- Users may override this array with any desired coefficients.
        -- These 16 values approximate a symmetric RRC pulse at roll-off 0.5.
        COEFF_0     : integer := 0;
        COEFF_1     : integer := -160;
        COEFF_2     : integer := -213;
        COEFF_3     : integer := 256;
        COEFF_4     : integer := 1108;
        COEFF_5     : integer := 2030;
        COEFF_6     : integer := 2561;
        COEFF_7     : integer := 2561;   -- centre (symmetric)
        COEFF_8     : integer := 2030;
        COEFF_9     : integer := 1108;
        COEFF_10    : integer := 256;
        COEFF_11    : integer := -213;
        COEFF_12    : integer := -160;
        COEFF_13    : integer := 0;
        COEFF_14    : integer := 0;
        COEFF_15    : integer := 0
    );
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;

        -- Input samples (signed fixed-point)
        in_valid : in  std_logic;
        in_i     : in  signed(DATA_WIDTH - 1 downto 0);
        in_q     : in  signed(DATA_WIDTH - 1 downto 0);

        -- Output filtered samples
        out_valid : out std_logic;
        out_i     : out signed(DATA_WIDTH - 1 downto 0);
        out_q     : out signed(DATA_WIDTH - 1 downto 0)
    );
end entity matched_filter;

architecture rtl of matched_filter is

    -- Coefficient type and array
    subtype coeff_t is signed(COEFF_WIDTH - 1 downto 0);
    type coeff_array_t is array (0 to NUM_TAPS - 1) of coeff_t;

    -- Build coefficient ROM from generics
    constant COEFFS : coeff_array_t := (
        0  => to_signed(COEFF_0,  COEFF_WIDTH),
        1  => to_signed(COEFF_1,  COEFF_WIDTH),
        2  => to_signed(COEFF_2,  COEFF_WIDTH),
        3  => to_signed(COEFF_3,  COEFF_WIDTH),
        4  => to_signed(COEFF_4,  COEFF_WIDTH),
        5  => to_signed(COEFF_5,  COEFF_WIDTH),
        6  => to_signed(COEFF_6,  COEFF_WIDTH),
        7  => to_signed(COEFF_7,  COEFF_WIDTH),
        8  => to_signed(COEFF_8,  COEFF_WIDTH),
        9  => to_signed(COEFF_9,  COEFF_WIDTH),
        10 => to_signed(COEFF_10, COEFF_WIDTH),
        11 => to_signed(COEFF_11, COEFF_WIDTH),
        12 => to_signed(COEFF_12, COEFF_WIDTH),
        13 => to_signed(COEFF_13, COEFF_WIDTH),
        14 => to_signed(COEFF_14, COEFF_WIDTH),
        15 => to_signed(COEFF_15, COEFF_WIDTH)
    );

    -- Shift-register delay lines for I and Q channels
    type sr_array_t is array (0 to NUM_TAPS - 1) of signed(DATA_WIDTH - 1 downto 0);
    signal sr_i : sr_array_t := (others => (others => '0'));
    signal sr_q : sr_array_t := (others => (others => '0'));

    -- MAC accumulator width: DATA_WIDTH + COEFF_WIDTH + log2(NUM_TAPS)
    constant ACC_WIDTH : positive := DATA_WIDTH + COEFF_WIDTH + 5; -- +5 covers up to 32 taps

    signal acc_i     : signed(ACC_WIDTH - 1 downto 0) := (others => '0');
    signal acc_q     : signed(ACC_WIDTH - 1 downto 0) := (others => '0');

    -- Pipeline registers
    signal valid_d1  : std_logic := '0';
    signal out_i_reg : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal out_q_reg : signed(DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Truncation: drop COEFF_WIDTH-1 LSBs (one bit kept as fractional guard)
    constant TRUNC_BITS : positive := COEFF_WIDTH - 1;

begin

    -- FIR filter process: shift register + MAC
    proc_fir : process(clk) is
        variable mac_i : signed(ACC_WIDTH - 1 downto 0);
        variable mac_q : signed(ACC_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                sr_i     <= (others => (others => '0'));
                sr_q     <= (others => (others => '0'));
                acc_i    <= (others => '0');
                acc_q    <= (others => '0');
                valid_d1 <= '0';
            elsif in_valid = '1' then
                -- Shift in new sample
                sr_i(1 to NUM_TAPS - 1) <= sr_i(0 to NUM_TAPS - 2);
                sr_q(1 to NUM_TAPS - 1) <= sr_q(0 to NUM_TAPS - 2);
                sr_i(0) <= in_i;
                sr_q(0) <= in_q;

                -- Multiply-accumulate over all taps
                mac_i := (others => '0');
                mac_q := (others => '0');
                for k in 0 to NUM_TAPS - 1 loop
                    mac_i := mac_i + resize(sr_i(k) * COEFFS(k), ACC_WIDTH);
                    mac_q := mac_q + resize(sr_q(k) * COEFFS(k), ACC_WIDTH);
                end loop;
                acc_i <= mac_i;
                acc_q <= mac_q;
            end if;
            valid_d1 <= in_valid;
        end if;
    end process proc_fir;

    -- Output pipeline register with truncation back to DATA_WIDTH
    proc_out : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                out_i_reg <= (others => '0');
                out_q_reg <= (others => '0');
            else
                -- Truncate: after shifting right by TRUNC_BITS, take the
                -- bottom DATA_WIDTH bits of the shifted result.
                out_i_reg <= resize(shift_right(acc_i, TRUNC_BITS)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
                out_q_reg <= resize(shift_right(acc_q, TRUNC_BITS)(DATA_WIDTH - 1 downto 0),
                                    DATA_WIDTH);
            end if;
        end if;
    end process proc_out;

    out_valid <= valid_d1;
    out_i     <= out_i_reg;
    out_q     <= out_q_reg;

end architecture rtl;
