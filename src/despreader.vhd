-- =============================================================================
-- Module: despreader.vhd
-- Description: PN despreader.  Multiplies incoming complex I/Q samples by the
--              PN chip value (+1/-1) and accumulates over one PN period
--              (integrate-and-dump).  Outputs one complex despread symbol per
--              PN period at symbol_valid.
--
--              When pn_chip = +1 : pass sample through unchanged
--              When pn_chip = -1 : negate sample (multiply by -1)
--
--              Bit-growth: accumulating PN_LENGTH samples of DATA_WIDTH bits
--              requires DATA_WIDTH + log2(PN_LENGTH) bits in the accumulator.
--              The output is truncated back to DATA_WIDTH.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity despreader is
    generic (
        DATA_WIDTH : positive := 16;    -- Width of I/Q input samples
        PN_LENGTH  : positive := 31     -- PN period in chips
    );
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;

        -- Input despread samples (from matched filter)
        in_valid     : in  std_logic;
        in_i         : in  signed(DATA_WIDTH - 1 downto 0);
        in_q         : in  signed(DATA_WIDTH - 1 downto 0);

        -- PN chip value from pn_generator
        pn_signed    : in  signed(1 downto 0);  -- +1 or -1

        -- Output: despread symbol (one per PN period)
        symbol_valid : out std_logic;
        symbol_i     : out signed(DATA_WIDTH - 1 downto 0);
        symbol_q     : out signed(DATA_WIDTH - 1 downto 0)
    );
end entity despreader;

architecture rtl of despreader is

    -- Accumulator width: DATA_WIDTH + ceil(log2(PN_LENGTH)) extra bits
    -- For PN_LENGTH=31 need 5 extra bits; use 6 for safety
    constant ACC_EXTRA : positive := 6;
    constant ACC_WIDTH : positive := DATA_WIDTH + ACC_EXTRA;

    signal acc_i    : signed(ACC_WIDTH - 1 downto 0) := (others => '0');
    signal acc_q    : signed(ACC_WIDTH - 1 downto 0) := (others => '0');
    signal chip_cnt : integer range 0 to PN_LENGTH - 1 := 0;

    signal sym_valid_r : std_logic := '0';
    signal sym_i_r     : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal sym_q_r     : signed(DATA_WIDTH - 1 downto 0) := (others => '0');

begin

    proc_despread : process(clk) is
        variable despread_i : signed(DATA_WIDTH - 1 downto 0);
        variable despread_q : signed(DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                acc_i      <= (others => '0');
                acc_q      <= (others => '0');
                chip_cnt   <= 0;
                sym_valid_r <= '0';
                sym_i_r    <= (others => '0');
                sym_q_r    <= (others => '0');
            else
                sym_valid_r <= '0';  -- Default: no new output

                if in_valid = '1' then
                    -- Multiply by PN chip: pn_signed = +1 or -1
                    if pn_signed = to_signed(1, 2) then
                        despread_i := in_i;
                        despread_q := in_q;
                    else
                        despread_i := -in_i;
                        despread_q := -in_q;
                    end if;

                    -- Accumulate (integrate)
                    acc_i <= acc_i + resize(despread_i, ACC_WIDTH);
                    acc_q <= acc_q + resize(despread_q, ACC_WIDTH);

                    if chip_cnt = PN_LENGTH - 1 then
                        -- Dump: output accumulated value, reset accumulator
                        -- Truncate by ACC_EXTRA bits to return to DATA_WIDTH
                        sym_i_r     <= resize(shift_right(acc_i + resize(despread_i, ACC_WIDTH),
                                              ACC_EXTRA), DATA_WIDTH);
                        sym_q_r     <= resize(shift_right(acc_q + resize(despread_q, ACC_WIDTH),
                                              ACC_EXTRA), DATA_WIDTH);
                        sym_valid_r <= '1';
                        acc_i       <= (others => '0');
                        acc_q       <= (others => '0');
                        chip_cnt    <= 0;
                    else
                        chip_cnt <= chip_cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process proc_despread;

    symbol_valid <= sym_valid_r;
    symbol_i     <= sym_i_r;
    symbol_q     <= sym_q_r;

end architecture rtl;
