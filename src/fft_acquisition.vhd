-- =============================================================================
-- Module: fft_acquisition.vhd
-- Description: FFT-based coarse acquisition stage.
--              Implements a Radix-2 Decimation-In-Time (DIT) FFT of
--              configurable size (FFT_SIZE must be a power of 2).
--
--              Twiddle factors are stored in a ROM.  Bit-reversal permutation
--              is applied to the input before butterfly processing.
--
--              After the FFT, magnitude is estimated as |I| + |Q| (a fast
--              approximation) for each bin.  The peak bin index and its
--              neighbour magnitudes are output for the 3-point frequency
--              estimator.
--
--              Pipeline stages:
--                Stage 0 : Input buffer + bit-reversal
--                Stages 1..LOG2N : butterfly stages (log2(FFT_SIZE) stages)
--                Stage LOG2N+1   : magnitude computation + peak detection
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity fft_acquisition is
    generic (
        DATA_WIDTH : positive := 16;   -- Input sample width
        FFT_SIZE   : positive := 64;   -- Must be a power of 2 (max 64 for synthesis)
        LOG2N      : positive := 6     -- log2(FFT_SIZE)
    );
    port (
        clk           : in  std_logic;
        rst           : in  std_logic;

        -- Input samples (time domain, despread symbols)
        in_valid      : in  std_logic;
        in_i          : in  signed(DATA_WIDTH - 1 downto 0);
        in_q          : in  signed(DATA_WIDTH - 1 downto 0);

        -- Output: peak bin and neighbour magnitudes for freq estimation
        out_valid     : out std_logic;
        peak_bin      : out unsigned(LOG2N - 1 downto 0);
        mag_peak      : out unsigned(DATA_WIDTH - 1 downto 0);
        mag_left      : out unsigned(DATA_WIDTH - 1 downto 0);
        mag_right     : out unsigned(DATA_WIDTH - 1 downto 0)
    );
end entity fft_acquisition;

architecture rtl of fft_acquisition is

    -- Internal word width after bit-growth through log2(N) butterfly stages
    constant INTERNAL_WIDTH : positive := DATA_WIDTH + LOG2N;

    -- Sample buffers (complex)
    type sample_array_t is array (0 to FFT_SIZE - 1) of signed(INTERNAL_WIDTH - 1 downto 0);

    signal buf_re    : sample_array_t := (others => (others => '0'));
    signal buf_im    : sample_array_t := (others => (others => '0'));

    -- State machine
    type state_t is (S_FILL, S_COMPUTE, S_PEAK);
    signal state : state_t := S_FILL;

    signal fill_idx   : integer range 0 to FFT_SIZE - 1 := 0;
    signal stage_idx  : integer range 0 to LOG2N        := 0;
    signal group_idx  : integer range 0 to FFT_SIZE / 2 - 1 := 0;

    -- Twiddle ROM: precomputed cos/sin at FFT_SIZE points
    -- Stored as INTERNAL_WIDTH signed fixed-point, scaled by 2^(INTERNAL_WIDTH-2)
    -- cos_table(k) = round(cos(2*pi*k/FFT_SIZE) * scale)
    -- Only one quarter-wave stored; full table reconstructed in logic
    constant TW_SCALE : real := real(2 ** (INTERNAL_WIDTH - 2));

    type twiddle_array_t is array (0 to FFT_SIZE / 2 - 1) of signed(INTERNAL_WIDTH - 1 downto 0);

    -- Twiddle factor generation function
    impure function gen_cos_table return twiddle_array_t is
        variable tbl : twiddle_array_t;
    begin
        for k in 0 to FFT_SIZE / 2 - 1 loop
            tbl(k) := to_signed(integer(round(
                cos(2.0 * MATH_PI * real(k) / real(FFT_SIZE)) * TW_SCALE)),
                INTERNAL_WIDTH);
        end loop;
        return tbl;
    end function;

    impure function gen_sin_table return twiddle_array_t is
        variable tbl : twiddle_array_t;
    begin
        for k in 0 to FFT_SIZE / 2 - 1 loop
            tbl(k) := to_signed(integer(round(
                (-sin(2.0 * MATH_PI * real(k) / real(FFT_SIZE))) * TW_SCALE)),
                INTERNAL_WIDTH);
        end loop;
        return tbl;
    end function;

    constant COS_TABLE : twiddle_array_t := gen_cos_table;
    constant SIN_TABLE : twiddle_array_t := gen_sin_table;

    -- Bit-reversal function
    function bit_reverse(val : unsigned; width : positive) return integer is
        variable reversed : unsigned(width - 1 downto 0) := (others => '0');
    begin
        for i in 0 to width - 1 loop
            reversed(i) := val(width - 1 - i);
        end loop;
        return to_integer(reversed);
    end function;

    -- Peak detection signals
    signal peak_bin_r  : integer range 0 to FFT_SIZE - 1 := 0;
    signal peak_mag_r  : unsigned(INTERNAL_WIDTH - 1 downto 0) := (others => '0');
    signal out_valid_r : std_logic := '0';
    signal search_idx  : integer range 0 to FFT_SIZE - 1 := 0;

    -- Magnitude computation
    signal mag_re : signed(INTERNAL_WIDTH - 1 downto 0);
    signal mag_im : signed(INTERNAL_WIDTH - 1 downto 0);

begin

    out_valid <= out_valid_r;

    -- Main FFT state machine
    proc_fft : process(clk) is
        variable tw_re   : signed(INTERNAL_WIDTH - 1 downto 0);
        variable tw_im   : signed(INTERNAL_WIDTH - 1 downto 0);
        variable a_re    : signed(INTERNAL_WIDTH - 1 downto 0);
        variable a_im    : signed(INTERNAL_WIDTH - 1 downto 0);
        variable b_re    : signed(INTERNAL_WIDTH - 1 downto 0);
        variable b_im    : signed(INTERNAL_WIDTH - 1 downto 0);
        variable t_re    : signed(2 * INTERNAL_WIDTH - 1 downto 0);
        variable t_im    : signed(2 * INTERNAL_WIDTH - 1 downto 0);
        variable bfly_re : signed(INTERNAL_WIDTH - 1 downto 0);
        variable bfly_im : signed(INTERNAL_WIDTH - 1 downto 0);
        variable half    : integer;
        variable stride  : integer;
        variable pair    : integer;
        variable tw_idx  : integer;
        variable mag_val : unsigned(INTERNAL_WIDTH - 1 downto 0);
        variable abs_re  : signed(INTERNAL_WIDTH - 1 downto 0);
        variable abs_im  : signed(INTERNAL_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state      <= S_FILL;
                fill_idx   <= 0;
                stage_idx  <= 0;
                group_idx  <= 0;
                search_idx <= 0;
                out_valid_r <= '0';
                peak_bin_r  <= 0;
                peak_mag_r  <= (others => '0');
                buf_re     <= (others => (others => '0'));
                buf_im     <= (others => (others => '0'));
            else
                out_valid_r <= '0';

                case state is
                    -- -------------------------------------------------------
                    -- S_FILL: collect FFT_SIZE input samples with bit reversal
                    -- -------------------------------------------------------
                    when S_FILL =>
                        if in_valid = '1' then
                            buf_re(bit_reverse(to_unsigned(fill_idx, LOG2N), LOG2N))
                                <= resize(in_i, INTERNAL_WIDTH);
                            buf_im(bit_reverse(to_unsigned(fill_idx, LOG2N), LOG2N))
                                <= resize(in_q, INTERNAL_WIDTH);
                            if fill_idx = FFT_SIZE - 1 then
                                fill_idx  <= 0;
                                stage_idx <= 0;
                                group_idx <= 0;
                                state     <= S_COMPUTE;
                            else
                                fill_idx <= fill_idx + 1;
                            end if;
                        end if;

                    -- -------------------------------------------------------
                    -- S_COMPUTE: butterfly stages (one butterfly per cycle)
                    -- -------------------------------------------------------
                    when S_COMPUTE =>
                        -- Current stage parameters
                        half   := 2 ** stage_idx;       -- half butterfly span
                        stride := 2 * half;              -- full butterfly span
                        pair   := group_idx / half * stride + group_idx mod half;
                        tw_idx := (group_idx mod half) * (FFT_SIZE / 2 / half);

                        -- Load butterfly inputs
                        a_re := buf_re(pair);
                        a_im := buf_im(pair);
                        b_re := buf_re(pair + half);
                        b_im := buf_im(pair + half);

                        -- Twiddle factor
                        tw_re := COS_TABLE(tw_idx);
                        tw_im := SIN_TABLE(tw_idx);

                        -- Complex multiply: t = b * tw
                        t_re := b_re * tw_re - b_im * tw_im;
                        t_im := b_re * tw_im + b_im * tw_re;

                        -- Scale down by INTERNAL_WIDTH-2 (remove twiddle scale)
                        bfly_re := resize(shift_right(t_re, INTERNAL_WIDTH - 2),
                                          INTERNAL_WIDTH);
                        bfly_im := resize(shift_right(t_im, INTERNAL_WIDTH - 2),
                                          INTERNAL_WIDTH);

                        -- Butterfly: A' = A + T, B' = A - T (with /2 scaling)
                        buf_re(pair)        <= shift_right(a_re + bfly_re, 1);
                        buf_im(pair)        <= shift_right(a_im + bfly_im, 1);
                        buf_re(pair + half) <= shift_right(a_re - bfly_re, 1);
                        buf_im(pair + half) <= shift_right(a_im - bfly_im, 1);

                        -- Advance
                        if group_idx = FFT_SIZE / 2 - 1 then
                            group_idx <= 0;
                            if stage_idx = LOG2N - 1 then
                                -- All stages done; move to peak detection
                                search_idx <= 0;
                                peak_mag_r <= (others => '0');
                                peak_bin_r <= 0;
                                state      <= S_PEAK;
                            else
                                stage_idx <= stage_idx + 1;
                            end if;
                        else
                            group_idx <= group_idx + 1;
                        end if;

                    -- -------------------------------------------------------
                    -- S_PEAK: compute |I|+|Q| magnitude, find peak bin
                    -- -------------------------------------------------------
                    when S_PEAK =>
                        abs_re := buf_re(search_idx) when buf_re(search_idx) >= 0
                                  else -buf_re(search_idx);
                        abs_im := buf_im(search_idx) when buf_im(search_idx) >= 0
                                  else -buf_im(search_idx);
                        mag_val := resize(unsigned(abs_re + abs_im),
                                          INTERNAL_WIDTH);

                        if mag_val > peak_mag_r then
                            peak_mag_r <= mag_val;
                            peak_bin_r <= search_idx;
                        end if;

                        if search_idx = FFT_SIZE - 1 then
                            out_valid_r <= '1';
                            -- Return to filling for next block
                            fill_idx  <= 0;
                            state     <= S_FILL;
                        else
                            search_idx <= search_idx + 1;
                        end if;
                end case;
            end if;
        end if;
    end process proc_fft;

    -- Output peak bin and neighbour magnitudes
    -- Registered combinatorially from buf arrays after peak detection
    proc_out : process(clk) is
        variable pk   : integer range 0 to FFT_SIZE - 1;
        variable left_idx  : integer range 0 to FFT_SIZE - 1;
        variable right_idx : integer range 0 to FFT_SIZE - 1;
        variable abs_re_l, abs_im_l : signed(INTERNAL_WIDTH - 1 downto 0);
        variable abs_re_r, abs_im_r : signed(INTERNAL_WIDTH - 1 downto 0);
        variable abs_re_p, abs_im_p : signed(INTERNAL_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                peak_bin  <= (others => '0');
                mag_peak  <= (others => '0');
                mag_left  <= (others => '0');
                mag_right <= (others => '0');
            elsif out_valid_r = '1' then
                pk := peak_bin_r;
                left_idx  := (pk - 1 + FFT_SIZE) mod FFT_SIZE;
                right_idx := (pk + 1) mod FFT_SIZE;

                abs_re_p := buf_re(pk)         when buf_re(pk) >= 0         else -buf_re(pk);
                abs_im_p := buf_im(pk)         when buf_im(pk) >= 0         else -buf_im(pk);
                abs_re_l := buf_re(left_idx)   when buf_re(left_idx) >= 0   else -buf_re(left_idx);
                abs_im_l := buf_im(left_idx)   when buf_im(left_idx) >= 0   else -buf_im(left_idx);
                abs_re_r := buf_re(right_idx)  when buf_re(right_idx) >= 0  else -buf_re(right_idx);
                abs_im_r := buf_im(right_idx)  when buf_im(right_idx) >= 0  else -buf_im(right_idx);

                peak_bin  <= to_unsigned(pk, LOG2N);
                mag_peak  <= resize(unsigned(abs_re_p + abs_im_p), DATA_WIDTH);
                mag_left  <= resize(unsigned(abs_re_l + abs_im_l), DATA_WIDTH);
                mag_right <= resize(unsigned(abs_re_r + abs_im_r), DATA_WIDTH);
            end if;
        end if;
    end process proc_out;

end architecture rtl;
