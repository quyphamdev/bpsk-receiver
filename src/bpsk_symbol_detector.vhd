-- =============================================================================
-- Module: bpsk_symbol_detector.vhd
-- Description: BPSK hard-decision symbol detector.
--              After carrier recovery the I channel carries the data; Q should
--              be near zero.  A bit decision is made based on the sign of I:
--                I >= 0  ->  bit = '1'
--                I <  0  ->  bit = '0'
--
--              Symbol timing: the module assumes the symbol rate is an integer
--              divisor of the sample rate.  A counter divides the sample clock
--              down to the symbol rate, and sampling occurs at the midpoint of
--              each symbol period (SAMPLES_PER_SYMBOL/2 offset).
--
--              A data_valid strobe marks each recovered bit.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bpsk_symbol_detector is
    generic (
        DATA_WIDTH          : positive := 16;
        SAMPLES_PER_SYMBOL  : positive := 4   -- Sample rate / symbol rate
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;

        -- Input: phase-corrected I/Q from Costas loop
        in_valid   : in  std_logic;
        in_i       : in  signed(DATA_WIDTH - 1 downto 0);
        in_q       : in  signed(DATA_WIDTH - 1 downto 0);

        -- Output: recovered bit with strobe
        data_valid : out std_logic;
        rx_bit     : out std_logic;

        -- Debug: raw I value at decision point
        dbg_i      : out signed(DATA_WIDTH - 1 downto 0)
    );
end entity bpsk_symbol_detector;

architecture rtl of bpsk_symbol_detector is

    -- Sample counter for symbol timing
    signal sample_cnt : integer range 0 to SAMPLES_PER_SYMBOL - 1 := 0;

    -- Sample point: midpoint of symbol
    constant SAMPLE_POINT : integer := SAMPLES_PER_SYMBOL / 2;

    signal data_valid_r : std_logic := '0';
    signal rx_bit_r     : std_logic := '0';
    signal dbg_i_r      : signed(DATA_WIDTH - 1 downto 0) := (others => '0');

begin

    proc_detect : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                sample_cnt  <= 0;
                data_valid_r <= '0';
                rx_bit_r    <= '0';
                dbg_i_r     <= (others => '0');
            else
                data_valid_r <= '0';  -- Default: no output

                if in_valid = '1' then
                    if sample_cnt = SAMPLE_POINT then
                        -- Decision: sign of I channel
                        if in_i >= 0 then
                            rx_bit_r <= '1';
                        else
                            rx_bit_r <= '0';
                        end if;
                        dbg_i_r      <= in_i;
                        data_valid_r <= '1';
                    end if;

                    if sample_cnt = SAMPLES_PER_SYMBOL - 1 then
                        sample_cnt <= 0;
                    else
                        sample_cnt <= sample_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process proc_detect;

    data_valid <= data_valid_r;
    rx_bit     <= rx_bit_r;
    dbg_i      <= dbg_i_r;

end architecture rtl;
