-- =============================================================================
-- Module: iq_input_interface.vhd
-- Description: Complex baseband I/Q sample input interface.
--              Implements an AXI-Stream style streaming interface with
--              valid/ready handshake. Provides synchronized, registered
--              output samples with 16-bit signed fixed-point representation.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity iq_input_interface is
    generic (
        DATA_WIDTH : positive := 16  -- Bit width of each I or Q sample
    );
    port (
        -- Clock and reset
        clk        : in  std_logic;
        rst        : in  std_logic;

        -- AXI-Stream slave input (raw samples from ADC / upstream)
        s_valid    : in  std_logic;
        s_ready    : out std_logic;
        s_i_data   : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
        s_q_data   : in  std_logic_vector(DATA_WIDTH - 1 downto 0);

        -- AXI-Stream master output (registered, downstream)
        m_valid    : out std_logic;
        m_ready    : in  std_logic;
        m_i_data   : out std_logic_vector(DATA_WIDTH - 1 downto 0);
        m_q_data   : out std_logic_vector(DATA_WIDTH - 1 downto 0)
    );
end entity iq_input_interface;

architecture rtl of iq_input_interface is

    -- Internal registered sample buffer
    signal buf_valid : std_logic := '0';
    signal buf_i     : std_logic_vector(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal buf_q     : std_logic_vector(DATA_WIDTH - 1 downto 0) := (others => '0');

begin

    -- s_ready: accept new data when buffer is empty or downstream is consuming
    s_ready <= (not buf_valid) or m_ready;

    -- Output assignments from the registered buffer
    m_valid  <= buf_valid;
    m_i_data <= buf_i;
    m_q_data <= buf_q;

    -- Register process: accept upstream sample when handshake completes
    proc_reg : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                buf_valid <= '0';
                buf_i     <= (others => '0');
                buf_q     <= (others => '0');
            else
                if s_valid = '1' and ((not buf_valid) or m_ready) = '1' then
                    -- Accept new sample from upstream
                    buf_valid <= '1';
                    buf_i     <= s_i_data;
                    buf_q     <= s_q_data;
                elsif m_ready = '1' and buf_valid = '1' then
                    -- Downstream consumed the sample; buffer is now empty
                    buf_valid <= '0';
                end if;
            end if;
        end if;
    end process proc_reg;

end architecture rtl;
