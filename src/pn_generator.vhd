-- =============================================================================
-- Module: pn_generator.vhd
-- Description: PN (pseudo-noise) sequence generator using a Linear Feedback
--              Shift Register (LFSR).  Configurable via generics:
--                PN_LENGTH   - number of LFSR stages (1..31)
--                POLYNOMIAL  - feedback tap mask (XOR feedback polynomial)
--                INIT_STATE  - initial seed / state load value
--
--              The LFSR runs at the chip rate (enabled by chip_en).  Output
--              pn_chip is the raw LFSR LSB; pn_signed maps 0->+1, 1->-1 for
--              easy complex multiplication.
--
--              Default polynomial: maximal-length for length 5 (0x12 = x^5+x^2+1)
--              but can be overridden.  Seed load is synchronous via load/seed
--              ports.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pn_generator is
    generic (
        PN_LENGTH  : positive := 31;        -- LFSR length (number of stages)
        POLYNOMIAL : std_logic_vector(30 downto 0) :=
            "0000000000000000000000000000101"; -- x^31+x^3+1: bits 0 and 2 set
        INIT_STATE : std_logic_vector(30 downto 0) :=
            "0000000000000000000000000000001"  -- Non-zero seed
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;

        -- Chip-rate clock enable: advance LFSR on each enabled cycle
        chip_en   : in  std_logic;

        -- Programmable phase offset: synchronous state load
        load      : in  std_logic;
        seed      : in  std_logic_vector(PN_LENGTH - 1 downto 0);

        -- PN output
        pn_chip   : out std_logic;                       -- Raw chip (0 or 1)
        pn_signed : out signed(1 downto 0)               -- +1 (chip=0) / -1 (chip=1)
    );
end entity pn_generator;

architecture rtl of pn_generator is

    signal lfsr : std_logic_vector(PN_LENGTH - 1 downto 0) :=
        INIT_STATE(PN_LENGTH - 1 downto 0);

    signal feedback : std_logic;

begin

    -- Compute XOR feedback from selected taps
    proc_feedback : process(lfsr) is
        variable fb : std_logic;
    begin
        fb := lfsr(PN_LENGTH - 1); -- always include MSB tap
        for i in 0 to PN_LENGTH - 2 loop
            if POLYNOMIAL(i) = '1' then
                fb := fb xor lfsr(i);
            end if;
        end loop;
        feedback <= fb;
    end process proc_feedback;

    -- LFSR register
    proc_lfsr : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                lfsr <= INIT_STATE(PN_LENGTH - 1 downto 0);
            elsif load = '1' then
                -- Synchronous state load for programmable phase offset
                lfsr <= seed;
            elsif chip_en = '1' then
                -- Shift left with feedback into LSB
                lfsr(PN_LENGTH - 1 downto 1) <= lfsr(PN_LENGTH - 2 downto 0);
                lfsr(0) <= feedback;
            end if;
        end if;
    end process proc_lfsr;

    -- Output the current MSB as chip
    pn_chip <= lfsr(PN_LENGTH - 1);

    -- Map chip to +1 / -1: chip=0 -> +1 (01), chip=1 -> -1 (11 in 2's complement)
    pn_signed <= to_signed(1, 2)  when lfsr(PN_LENGTH - 1) = '0' else
                 to_signed(-1, 2);

end architecture rtl;
