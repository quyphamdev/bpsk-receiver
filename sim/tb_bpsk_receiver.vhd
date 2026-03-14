-- =============================================================================
-- Testbench: tb_bpsk_receiver.vhd
-- Description: Full-system testbench for the BPSK spread-spectrum receiver.
--
--   The testbench:
--     1. Generates a known BPSK bit pattern
--     2. Applies PN spreading using the same PN sequence as the receiver
--     3. Applies a carrier frequency offset (CFO)
--     4. Adds simulated white Gaussian noise using Box-Muller transform
--     5. Feeds the signal into top_level_receiver
--     6. Captures recovered bits and checks against the transmitted sequence
--        (with allowance for acquisition delay)
--     7. Reports pass/fail via VHDL assert + report statements
--
--  Configurable parameters:
--    SIM_SYMBOLS       - number of BPSK symbols to transmit
--    SNR_DB            - approximate SNR in dB (scales noise amplitude)
--    CFO_CYCLES_PER_FS - frequency offset expressed as cycles per sample period
--                        (0.01 = 1% of sample rate)
--
--  ModelSim usage:
--    vcom -2008 src/*.vhd sim/tb_bpsk_receiver.vhd
--    vsim tb_bpsk_receiver
--    run -all
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.env.all;  -- VHDL-2008: provides finish/stop

entity tb_bpsk_receiver is
    generic (
        SIM_SYMBOLS         : positive := 256;    -- Number of BPSK symbols
        SNR_DB              : real     := 20.0;   -- Signal-to-noise ratio (dB)
        CFO_NORM            : real     := 0.005;  -- Normalised CFO (fraction of fs)
        SAMPLES_PER_SYMBOL  : positive := 1;      -- Samples per PN chip (1 = no oversampling)
        PN_LENGTH           : positive := 31;
        DATA_WIDTH          : positive := 16;
        FFT_SIZE            : positive := 64;
        LOG2_FFT            : positive := 6
    );
end entity tb_bpsk_receiver;

architecture sim of tb_bpsk_receiver is

    -- =========================================================================
    -- DUT component
    -- =========================================================================
    component top_level_receiver is
        generic (
            DATA_WIDTH           : positive;
            FFT_SIZE             : positive;
            LOG2_FFT             : positive;
            PN_LENGTH            : positive;
            PN_POLYNOMIAL        : std_logic_vector(30 downto 0);
            NUM_TAPS             : positive;
            SAMPLES_PER_SYMBOL   : positive;
            COSTAS_KP            : integer;
            COSTAS_KI            : integer;
            COSTAS_GAIN_SHIFT    : positive;
            LUT_DEPTH            : positive
        );
        port (
            clk            : in  std_logic;
            rst            : in  std_logic;
            s_valid        : in  std_logic;
            s_ready        : out std_logic;
            s_i_data       : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
            s_q_data       : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
            data_valid     : out std_logic;
            rx_bit         : out std_logic;
            locked         : out std_logic;
            dbg_mf_valid   : out std_logic;
            dbg_mf_i       : out signed(DATA_WIDTH - 1 downto 0);
            dbg_mf_q       : out signed(DATA_WIDTH - 1 downto 0);
            dbg_fft_valid  : out std_logic;
            dbg_peak_bin   : out unsigned(LOG2_FFT - 1 downto 0);
            dbg_freq_est   : out signed(LOG2_FFT + 8 downto 0);
            dbg_costas_i   : out signed(DATA_WIDTH - 1 downto 0);
            dbg_costas_q   : out signed(DATA_WIDTH - 1 downto 0);
            dbg_desp_valid : out std_logic;
            dbg_desp_i     : out signed(DATA_WIDTH - 1 downto 0);
            dbg_desp_q     : out signed(DATA_WIDTH - 1 downto 0)
        );
    end component;

    -- =========================================================================
    -- Clock and reset
    -- =========================================================================
    constant CLK_PERIOD : time := 10 ns;  -- 100 MHz

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    -- =========================================================================
    -- DUT I/O
    -- =========================================================================
    signal s_valid    : std_logic := '0';
    signal s_ready    : std_logic;
    signal s_i_data   : std_logic_vector(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_q_data   : std_logic_vector(DATA_WIDTH - 1 downto 0) := (others => '0');

    signal data_valid : std_logic;
    signal rx_bit     : std_logic;
    signal locked     : std_logic;

    signal dbg_mf_valid   : std_logic;
    signal dbg_mf_i       : signed(DATA_WIDTH - 1 downto 0);
    signal dbg_mf_q       : signed(DATA_WIDTH - 1 downto 0);
    signal dbg_fft_valid  : std_logic;
    signal dbg_peak_bin   : unsigned(LOG2_FFT - 1 downto 0);
    signal dbg_freq_est   : signed(LOG2_FFT + 8 downto 0);
    signal dbg_costas_i   : signed(DATA_WIDTH - 1 downto 0);
    signal dbg_costas_q   : signed(DATA_WIDTH - 1 downto 0);
    -- Despreader output: observe PN correlation peak in Vivado waveform viewer
    signal dbg_desp_valid : std_logic;
    signal dbg_desp_i     : signed(DATA_WIDTH - 1 downto 0);
    signal dbg_desp_q     : signed(DATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Simulation helpers
    -- =========================================================================
    constant SIG_AMPLITUDE : real    := 8192.0;  -- Peak amplitude (16-bit range = 32767)
    constant NOISE_SIGMA   : real    := SIG_AMPLITUDE /
        (10.0 ** (SNR_DB / 20.0));               -- Noise std deviation from SNR

    -- Total chips in the simulation
    constant TOTAL_CHIPS   : integer := SIM_SYMBOLS * PN_LENGTH;
    constant TOTAL_SAMPLES : integer := TOTAL_CHIPS * SAMPLES_PER_SYMBOL;

    -- Bit pattern storage
    type bit_array_t   is array (0 to SIM_SYMBOLS - 1) of std_logic;
    type int_array_t   is array (0 to TOTAL_SAMPLES - 1) of integer;

    -- -------------------------------------------------------------------------
    -- Pseudo-random helper functions
    -- -------------------------------------------------------------------------

    -- Box-Muller: generate a normally distributed value from two uniform [0,1] samples
    -- Returns scaled integer noise sample
    procedure box_muller(
        u1     : in  real;
        u2     : in  real;
        sigma  : in  real;
        result : out real
    ) is
        variable z0 : real;
    begin
        z0 := sigma * sqrt(-2.0 * log(u1 + 1.0e-10)) * cos(MATH_2_PI * u2);
        result := z0;
    end procedure;

    -- Maximal-length LFSR for PN sequence (same polynomial as receiver)
    function lfsr_next(state : std_logic_vector(30 downto 0))
        return std_logic_vector is
        variable fb    : std_logic;
        variable nxt   : std_logic_vector(30 downto 0);
    begin
        -- Polynomial: x^31+x^3+1 feedback taps at positions 0 and 2 (0-indexed)
        fb := state(30) xor state(2) xor state(0);
        nxt(30 downto 1) := state(29 downto 0);
        nxt(0) := fb;
        return nxt;
    end function;

    -- =========================================================================
    -- Simulation test sequence
    -- =========================================================================

    -- Transmitted bits
    signal tx_bits : bit_array_t := (others => '0');

    -- Received bits
    signal rx_bits  : bit_array_t := (others => '0');
    signal rx_count : integer range 0 to SIM_SYMBOLS := 0;

begin

    -- =========================================================================
    -- Clock generation
    -- =========================================================================
    clk <= not clk after CLK_PERIOD / 2;

    -- =========================================================================
    -- DUT instantiation
    -- =========================================================================
    u_dut : top_level_receiver
        generic map (
            DATA_WIDTH         => DATA_WIDTH,
            FFT_SIZE           => FFT_SIZE,
            LOG2_FFT           => LOG2_FFT,
            PN_LENGTH          => PN_LENGTH,
            PN_POLYNOMIAL      => "0000000000000000000000000000101",
            NUM_TAPS           => 16,
            SAMPLES_PER_SYMBOL => SAMPLES_PER_SYMBOL,
            COSTAS_KP          => 128,
            COSTAS_KI          => 0,   -- Pure proportional: no integrator overflow at symbol rate
            COSTAS_GAIN_SHIFT  => 7,   -- K_eff=0.38 with PHASE_WIDTH=16, A≈3968 (symbol rate)
            LUT_DEPTH          => 256
        )
        port map (
            clk            => clk,
            rst            => rst,
            s_valid        => s_valid,
            s_ready        => s_ready,
            s_i_data       => s_i_data,
            s_q_data       => s_q_data,
            data_valid     => data_valid,
            rx_bit         => rx_bit,
            locked         => locked,
            dbg_mf_valid   => dbg_mf_valid,
            dbg_mf_i       => dbg_mf_i,
            dbg_mf_q       => dbg_mf_q,
            dbg_fft_valid  => dbg_fft_valid,
            dbg_peak_bin   => dbg_peak_bin,
            dbg_freq_est   => dbg_freq_est,
            dbg_costas_i   => dbg_costas_i,
            dbg_costas_q   => dbg_costas_q,
            dbg_desp_valid => dbg_desp_valid,
            dbg_desp_i     => dbg_desp_i,
            dbg_desp_q     => dbg_desp_q
        );

    -- =========================================================================
    -- Stimulus process: generate BPSK+PN spread signal with CFO and noise
    -- =========================================================================
    proc_stim : process is
        variable seed1, seed2  : positive := 42;
        variable uniform1, uniform2 : real;
        variable noise_i, noise_q   : real;

        variable pn_state    : std_logic_vector(30 downto 0) :=
            "0000000000000000000000000000001";
        variable pn_chip_val : std_logic;

        variable bpsk_sym    : real;   -- +1 or -1
        variable phase       : real := 0.0;
        variable cfo_phase   : real := 0.0;
        variable bit_rand    : real;   -- uniform random for bit generation

        variable sample_i    : integer;
        variable sample_q    : integer;
        variable chip_idx    : integer;
        variable sym_idx     : integer;
        variable samp_idx    : integer;
    begin
        -- Reset for 10 cycles
        rst     <= '1';
        s_valid <= '0';
        wait for CLK_PERIOD * 10;
        rst <= '0';
        wait for CLK_PERIOD * 2;

        -- -----------------------------------------------------------------
        -- Generate and transmit signal
        -- -----------------------------------------------------------------
        chip_idx := 0;
        sym_idx  := 0;

        for sym in 0 to SIM_SYMBOLS - 1 loop
            -- Generate random BPSK bit using uniform PRNG
            uniform(seed1, seed2, bit_rand);
            if bit_rand < 0.5 then
                tx_bits(sym) <= '0';
                bpsk_sym := -1.0;
            else
                tx_bits(sym) <= '1';
                bpsk_sym := 1.0;
            end if;

            -- Spread over PN_LENGTH chips
            for chip in 0 to PN_LENGTH - 1 loop
                pn_chip_val := pn_state(30);

                -- PN modulation: chip=0 -> +1, chip=1 -> -1
                if pn_chip_val = '0' then
                    bpsk_sym := bpsk_sym;   -- multiply by +1
                else
                    bpsk_sym := -bpsk_sym;  -- multiply by -1 (negate)
                end if;

                -- Advance LFSR
                pn_state := lfsr_next(pn_state);

                -- Send SAMPLES_PER_SYMBOL samples per chip
                for samp in 0 to SAMPLES_PER_SYMBOL - 1 loop
                    -- Apply carrier frequency offset
                    cfo_phase := cfo_phase + MATH_2_PI * CFO_NORM;

                    -- Complex baseband sample with CFO
                    -- I = bpsk_sym * cos(cfo_phase), Q = bpsk_sym * sin(cfo_phase)
                    uniform1 := 0.0; uniform2 := 0.0;
                    uniform(seed1, seed2, uniform1);
                    uniform(seed1, seed2, uniform2);
                    box_muller(uniform1, uniform2, NOISE_SIGMA, noise_i);
                    uniform(seed1, seed2, uniform1);
                    uniform(seed1, seed2, uniform2);
                    box_muller(uniform1, uniform2, NOISE_SIGMA, noise_q);

                    sample_i := integer(SIG_AMPLITUDE * bpsk_sym * cos(cfo_phase) + noise_i);
                    sample_q := integer(SIG_AMPLITUDE * bpsk_sym * sin(cfo_phase) + noise_q);

                    -- Clamp to 16-bit range
                    if sample_i >  32767 then sample_i :=  32767; end if;
                    if sample_i < -32768 then sample_i := -32768; end if;
                    if sample_q >  32767 then sample_q :=  32767; end if;
                    if sample_q < -32768 then sample_q := -32768; end if;

                    -- Drive DUT inputs
                    wait until rising_edge(clk);
                    s_valid  <= '1';
                    s_i_data <= std_logic_vector(to_signed(sample_i, DATA_WIDTH));
                    s_q_data <= std_logic_vector(to_signed(sample_q, DATA_WIDTH));
                end loop;

                -- Restore bpsk_sym polarity for next chip
                if pn_chip_val = '1' then
                    bpsk_sym := -bpsk_sym;
                end if;
            end loop;
        end loop;

        -- Send a few more idle cycles
        wait until rising_edge(clk);
        s_valid <= '0';
        s_i_data <= (others => '0');
        s_q_data <= (others => '0');

        -- Wait for pipeline drain
        wait for CLK_PERIOD * 500;

        -- Simulation done
        wait;
    end process proc_stim;

    -- =========================================================================
    -- Capture recovered bits
    -- =========================================================================
    proc_capture : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                rx_count <= 0;
            elsif data_valid = '1' then
                if rx_count < SIM_SYMBOLS then
                    rx_bits(rx_count) <= rx_bit;
                    rx_count <= rx_count + 1;
                end if;
            end if;
        end if;
    end process proc_capture;

    -- =========================================================================
    -- Monitor and check process
    -- =========================================================================
    proc_check : process is
        variable errors       : integer := 0;
        variable checked      : integer := 0;
        variable err_norm     : integer := 0;
        variable err_inv      : integer := 0;
        -- Skip the first ACQ_SKIP symbols to allow the receiver to acquire:
        --   * FFT fills after FFT_SIZE symbols (coarse frequency correction)
        --   * Costas loop converges after ~30 more symbols
        -- 100 symbols is a safe margin for FFT_SIZE <= 64.
        constant ACQ_SKIP     : integer := 100;
        variable polarity_inv : boolean := false;
        variable expected_bit : std_logic;
    begin
        -- Wait until enough bits have been received
        wait until rx_count = SIM_SYMBOLS;

        -- Wait one more clock for final bit
        wait until rising_edge(clk);

        -- -----------------------------------------------------------------
        -- Check polarity: BPSK is inherently 180-deg ambiguous; detect by
        -- comparing a window of bits and picking the polarity with fewer errors
        -- -----------------------------------------------------------------
        err_norm := 0;
        err_inv  := 0;
        for i in ACQ_SKIP to SIM_SYMBOLS - 1 loop
            if tx_bits(i) /= rx_bits(i) then
                err_norm := err_norm + 1;
            else
                err_inv := err_inv + 1;
            end if;
        end loop;
        polarity_inv := (err_inv < err_norm);

        -- -----------------------------------------------------------------
        -- Count BER (excluding acquisition window)
        -- -----------------------------------------------------------------
        errors  := 0;
        checked := 0;
        for i in ACQ_SKIP to SIM_SYMBOLS - 1 loop
            if polarity_inv then
                expected_bit := not tx_bits(i);
            else
                expected_bit := tx_bits(i);
            end if;

            if rx_bits(i) /= expected_bit then
                errors := errors + 1;
            end if;
            checked := checked + 1;
        end loop;

        -- -----------------------------------------------------------------
        -- Report results
        -- -----------------------------------------------------------------
        report "=== BPSK Receiver Testbench Results ===" severity note;
        report "  Symbols transmitted : " & integer'image(SIM_SYMBOLS) severity note;
        report "  Acquisition skip    : " & integer'image(ACQ_SKIP) & " symbols" severity note;
        report "  Symbols checked     : " & integer'image(checked) severity note;
        report "  Bit errors          : " & integer'image(errors) severity note;
        report "  BER                 : " &
               integer'image((errors * 1000) / (checked + 1)) & "/1000" severity note;
        report "  SNR (dB)            : " & real'image(SNR_DB) severity note;
        report "  CFO (norm)          : " & real'image(CFO_NORM) severity note;
        if locked = '1' then
            report "  Carrier lock        : YES (Costas loop locked)" severity note;
        else
            report "  Carrier lock        : NO  (Costas loop not locked)" severity note;
        end if;
        if polarity_inv then
            report "  Phase polarity      : INVERTED (180-deg ambiguity resolved)" severity note;
        else
            report "  Phase polarity      : NORMAL" severity note;
        end if;

        -- Show first 8 post-acquisition bit comparisons for quick visual check
        report "  TX bits [ACQ_SKIP..ACQ_SKIP+7]: " &
               std_logic'image(tx_bits(ACQ_SKIP+0)) &
               std_logic'image(tx_bits(ACQ_SKIP+1)) &
               std_logic'image(tx_bits(ACQ_SKIP+2)) &
               std_logic'image(tx_bits(ACQ_SKIP+3)) &
               std_logic'image(tx_bits(ACQ_SKIP+4)) &
               std_logic'image(tx_bits(ACQ_SKIP+5)) &
               std_logic'image(tx_bits(ACQ_SKIP+6)) &
               std_logic'image(tx_bits(ACQ_SKIP+7)) severity note;
        report "  RX bits [ACQ_SKIP..ACQ_SKIP+7]: " &
               std_logic'image(rx_bits(ACQ_SKIP+0)) &
               std_logic'image(rx_bits(ACQ_SKIP+1)) &
               std_logic'image(rx_bits(ACQ_SKIP+2)) &
               std_logic'image(rx_bits(ACQ_SKIP+3)) &
               std_logic'image(rx_bits(ACQ_SKIP+4)) &
               std_logic'image(rx_bits(ACQ_SKIP+5)) &
               std_logic'image(rx_bits(ACQ_SKIP+6)) &
               std_logic'image(rx_bits(ACQ_SKIP+7)) severity note;

        -- BER check: require < 10% BER in the post-acquisition window.
        assert errors * 10 < checked
            report "FAIL: BER is high (" & integer'image(errors) & "/" &
                   integer'image(checked) &
                   " errors) - PN acquisition may not have locked"
            severity failure;

        -- Verify that the DUT produced output (data_valid pulsed at least once)
        assert rx_count > 0
            report "FAIL: No bits received from DUT"
            severity failure;

        report "SIMULATION COMPLETE: DUT produced " & integer'image(rx_count) &
               " bits. BER = " &
               integer'image((errors * 100) / (checked + 1)) & "%" severity note;

        -- Cleanly end the simulation (VHDL-2008)
        finish;
        wait;
    end process proc_check;

    -- =========================================================================
    -- Debug monitor: print FFT/FreqEst events and lock events during simulation
    -- =========================================================================
    proc_debug : process is
        variable fft_count : integer := 0;
    begin
        wait until rst = '0';
        loop
            -- Report each FFT acquisition result
            wait until rising_edge(clk) and dbg_fft_valid = '1';
            fft_count := fft_count + 1;
            report "FFT#" & integer'image(fft_count) &
                   "  peak_bin=" & integer'image(to_integer(dbg_peak_bin)) &
                   "  freq_est=" & integer'image(to_integer(dbg_freq_est)) &
                   "  rx_count=" & integer'image(rx_count)
                   severity note;
        end loop;
    end process proc_debug;

    proc_debug_lock : process is
    begin
        wait until rst = '0';
        -- Report when Costas loop locks for the first time
        wait until locked = '1';
        report "Costas loop LOCKED at rx_count=" & integer'image(rx_count)
               severity note;
        wait;
    end process proc_debug_lock;

    -- =========================================================================
    -- Timeout watchdog
    -- =========================================================================
    proc_watchdog : process is
    begin
        wait for CLK_PERIOD * (TOTAL_SAMPLES + 10000);
        report "TIMEOUT: Simulation exceeded expected duration." severity failure;
        wait;
    end process proc_watchdog;

end architecture sim;
