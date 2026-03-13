-- =============================================================================
-- Module: frequency_estimator_3point.vhd
-- Description: 3-point parabolic interpolation frequency estimator.
--              Uses the peak FFT bin and its two neighbours to compute a
--              sub-bin frequency offset.
--
--              Formula:
--                delta = 0.5 * (M[k+1] - M[k-1]) / (2*M[k] - M[k-1] - M[k+1])
--
--              The division is performed using an iterative non-restoring
--              divider (parameterised width).  The output frequency offset
--              is expressed as a signed fixed-point value in units of
--              (fs / FFT_SIZE) per LSB, scaled by 2^FRAC_BITS.
--
--              Latency: DIVIDER_WIDTH+3 clock cycles after in_valid.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity frequency_estimator_3point is
    generic (
        MAG_WIDTH    : positive := 16;  -- Width of magnitude inputs
        FFT_SIZE     : positive := 64;
        LOG2N        : positive := 6;   -- log2(FFT_SIZE)
        FRAC_BITS    : positive := 8;   -- Fractional bits in delta output
        DIV_WIDTH    : positive := 24   -- Divider precision width
    );
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;

        -- Inputs from FFT acquisition block
        in_valid     : in  std_logic;
        peak_bin     : in  unsigned(LOG2N - 1 downto 0);
        mag_peak     : in  unsigned(MAG_WIDTH - 1 downto 0);  -- M[k]
        mag_left     : in  unsigned(MAG_WIDTH - 1 downto 0);  -- M[k-1]
        mag_right    : in  unsigned(MAG_WIDTH - 1 downto 0);  -- M[k+1]

        -- Output frequency estimate
        out_valid    : out std_logic;
        freq_offset  : out signed(LOG2N + FRAC_BITS downto 0)  -- signed fixed-point
    );
end entity frequency_estimator_3point;

architecture rtl of frequency_estimator_3point is

    -- -------------------------------------------------------------------------
    -- Pipeline stage 1: compute numerator and denominator
    -- numerator   = 0.5 * (M[k+1] - M[k-1])  -> stored as (M[k+1]-M[k-1])
    -- denominator = 2*M[k] - M[k-1] - M[k+1] -> always >= 0 for a true peak
    -- -------------------------------------------------------------------------
    signal s1_valid    : std_logic := '0';
    signal s1_peak_bin : unsigned(LOG2N - 1 downto 0) := (others => '0');
    signal s1_numer    : signed(MAG_WIDTH downto 0)   := (others => '0');  -- can be negative
    signal s1_denom    : unsigned(MAG_WIDTH downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- Iterative divider signals
    -- -------------------------------------------------------------------------
    signal div_start   : std_logic := '0';
    signal div_busy    : std_logic := '0';
    signal div_done    : std_logic := '0';
    signal div_numer   : signed(DIV_WIDTH - 1 downto 0)   := (others => '0');
    signal div_denom   : unsigned(DIV_WIDTH - 1 downto 0) := (others => '0');
    signal div_quot    : signed(DIV_WIDTH - 1 downto 0)   := (others => '0');
    signal div_sign    : std_logic := '0';  -- sign of numerator
    signal div_cnt     : integer range 0 to DIV_WIDTH + 1 := 0;
    signal div_rem     : unsigned(DIV_WIDTH downto 0)     := (others => '0');
    signal div_q       : unsigned(DIV_WIDTH - 1 downto 0) := (others => '0');
    signal peak_bin_d  : unsigned(LOG2N - 1 downto 0)     := (others => '0');

begin

    -- -------------------------------------------------------------------------
    -- Stage 1: register inputs and compute num/denom
    -- -------------------------------------------------------------------------
    proc_s1 : process(clk) is
        variable mk_minus : unsigned(MAG_WIDTH downto 0);
        variable mk_plus  : unsigned(MAG_WIDTH downto 0);
        variable mk       : unsigned(MAG_WIDTH downto 0);
        variable num      : signed(MAG_WIDTH downto 0);
        variable den      : unsigned(MAG_WIDTH downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                s1_valid    <= '0';
                s1_peak_bin <= (others => '0');
                s1_numer    <= (others => '0');
                s1_denom    <= (others => '0');
            else
                s1_valid <= in_valid;
                if in_valid = '1' then
                    mk_minus := resize(mag_left,  MAG_WIDTH + 1);
                    mk_plus  := resize(mag_right, MAG_WIDTH + 1);
                    mk       := resize(mag_peak,  MAG_WIDTH + 1);

                    -- numerator = M[k+1] - M[k-1]  (can be negative)
                    num := signed(mk_plus) - signed(mk_minus);

                    -- denominator = 2*M[k] - M[k-1] - M[k+1]  (>= 0 at peak)
                    den := shift_left(mk, 1) - mk_minus - mk_plus;
                    if den = 0 then
                        den := to_unsigned(1, MAG_WIDTH + 1);  -- avoid div-by-zero
                    end if;

                    s1_peak_bin <= peak_bin;
                    s1_numer    <= num;
                    s1_denom    <= den;
                end if;
            end if;
        end if;
    end process proc_s1;

    -- -------------------------------------------------------------------------
    -- Stage 2: launch iterative divider when stage-1 fires
    -- Result: delta_scaled = (numer << FRAC_BITS) / denom
    --         then freq_offset = peak_bin + delta/2  (the /2 from the formula)
    -- -------------------------------------------------------------------------
    proc_div : process(clk) is
        variable abs_numer : unsigned(DIV_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                div_busy   <= '0';
                div_done   <= '0';
                div_cnt    <= 0;
                div_rem    <= (others => '0');
                div_q      <= (others => '0');
                div_sign   <= '0';
                peak_bin_d <= (others => '0');
            else
                div_done <= '0';

                if s1_valid = '1' and div_busy = '0' then
                    -- Start division: compute |numer| << FRAC_BITS
                    div_sign <= s1_numer(s1_numer'left);
                    if s1_numer(s1_numer'left) = '1' then
                        abs_numer := resize(unsigned(-s1_numer), DIV_WIDTH);
                    else
                        abs_numer := resize(unsigned(s1_numer), DIV_WIDTH);
                    end if;
                    -- Scale numerator by 2^FRAC_BITS
                    div_rem    <= resize(shift_left(abs_numer, FRAC_BITS), DIV_WIDTH + 1);
                    div_denom  <= resize(s1_denom, DIV_WIDTH);
                    div_q      <= (others => '0');
                    div_cnt    <= DIV_WIDTH;
                    div_busy   <= '1';
                    peak_bin_d <= s1_peak_bin;

                elsif div_busy = '1' then
                    -- Non-restoring shift division: one bit per cycle
                    div_rem <= shift_left(div_rem, 1);
                    if shift_left(div_rem, 1)(DIV_WIDTH downto 1) >= div_denom then
                        div_q   <= shift_left(div_q, 1) or to_unsigned(1, DIV_WIDTH);
                        div_rem(DIV_WIDTH downto 1) <=
                            shift_left(div_rem, 1)(DIV_WIDTH downto 1) - div_denom;
                    else
                        div_q <= shift_left(div_q, 1);
                    end if;

                    if div_cnt = 1 then
                        div_busy <= '0';
                        div_done <= '1';
                    else
                        div_cnt <= div_cnt - 1;
                    end if;
                end if;
            end if;
        end if;
    end process proc_div;

    -- -------------------------------------------------------------------------
    -- Stage 3: assemble output frequency offset
    -- freq_offset = peak_bin * 2^FRAC_BITS ± delta/2
    -- where delta = div_q (in units of 1/2^FRAC_BITS)
    -- The /2 factor from the formula is handled by right-shifting div_q by 1.
    -- -------------------------------------------------------------------------
    proc_out : process(clk) is
        variable base   : signed(LOG2N + FRAC_BITS downto 0);
        variable delta  : signed(LOG2N + FRAC_BITS downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                out_valid   <= '0';
                freq_offset <= (others => '0');
            else
                out_valid <= '0';
                if div_done = '1' then
                    -- Base: peak_bin scaled by 2^FRAC_BITS
                    base  := resize(shift_left(signed(resize(peak_bin_d, LOG2N + FRAC_BITS + 1)),
                                               FRAC_BITS),
                                    LOG2N + FRAC_BITS + 1);
                    -- Delta/2 in fixed-point
                    delta := resize(signed('0' & shift_right(div_q, 1)), LOG2N + FRAC_BITS + 1);

                    if div_sign = '1' then
                        freq_offset <= base - delta;
                    else
                        freq_offset <= base + delta;
                    end if;
                    out_valid <= '1';
                end if;
            end if;
        end if;
    end process proc_out;

end architecture rtl;
