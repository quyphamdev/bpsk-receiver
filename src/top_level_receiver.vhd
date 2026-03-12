-- =============================================================================
-- Module: top_level_receiver.vhd
-- Description: Top-level interconnect for the BPSK spread-spectrum receiver.
--              Instantiates and wires together all sub-modules in signal-flow
--              order:
--
--    IQ Input Interface
--         |
--    Matched Filter
--         |
--    PN Generator (chip clock) + Despreader
--         |
--    FFT Acquisition
--         |
--    3-Point Frequency Estimator  -->  NCO Frequency Corrector
--                                             |
--                                      Costas Loop
--                                             |
--                                      Symbol Detector  --> rx_bit / data_valid
--
--  Debug ports expose intermediate signals for ModelSim waveform inspection.
-- =============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity top_level_receiver is
    generic (
        DATA_WIDTH           : positive := 16;
        FFT_SIZE             : positive := 64;
        LOG2_FFT             : positive := 6;
        PN_LENGTH            : positive := 31;
        PN_POLYNOMIAL        : std_logic_vector(30 downto 0) :=
            "0000000000000000000000000000101";
        NUM_TAPS             : positive := 16;
        SAMPLES_PER_SYMBOL   : positive := 4;
        -- Costas loop gains
        COSTAS_KP            : integer  := 128;
        COSTAS_KI            : integer  := 4;
        COSTAS_GAIN_SHIFT    : positive := 12;
        -- NCO/Costas LUT depth
        LUT_DEPTH            : positive := 256
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        -- =====================================================================
        -- I/Q Input (AXI-Stream slave)
        -- =====================================================================
        s_valid     : in  std_logic;
        s_ready     : out std_logic;
        s_i_data    : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
        s_q_data    : in  std_logic_vector(DATA_WIDTH - 1 downto 0);

        -- =====================================================================
        -- Bit output
        -- =====================================================================
        data_valid  : out std_logic;
        rx_bit      : out std_logic;
        locked      : out std_logic;

        -- =====================================================================
        -- Debug / monitor ports
        -- =====================================================================
        dbg_mf_valid  : out std_logic;
        dbg_mf_i      : out signed(DATA_WIDTH - 1 downto 0);
        dbg_mf_q      : out signed(DATA_WIDTH - 1 downto 0);
        dbg_fft_valid : out std_logic;
        dbg_peak_bin  : out unsigned(LOG2_FFT - 1 downto 0);
        dbg_freq_est  : out signed(LOG2_FFT + 8 downto 0);  -- 8 frac bits
        dbg_costas_i  : out signed(DATA_WIDTH - 1 downto 0);
        dbg_costas_q  : out signed(DATA_WIDTH - 1 downto 0)
    );
end entity top_level_receiver;

architecture rtl of top_level_receiver is

    -- =========================================================================
    -- Component declarations
    -- =========================================================================

    component iq_input_interface is
        generic (DATA_WIDTH : positive);
        port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            s_valid  : in  std_logic;
            s_ready  : out std_logic;
            s_i_data : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
            s_q_data : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
            m_valid  : out std_logic;
            m_ready  : in  std_logic;
            m_i_data : out std_logic_vector(DATA_WIDTH - 1 downto 0);
            m_q_data : out std_logic_vector(DATA_WIDTH - 1 downto 0)
        );
    end component;

    component matched_filter is
        generic (
            DATA_WIDTH  : positive;
            COEFF_WIDTH : positive;
            NUM_TAPS    : positive;
            COEFF_0     : integer; COEFF_1  : integer; COEFF_2  : integer;
            COEFF_3     : integer; COEFF_4  : integer; COEFF_5  : integer;
            COEFF_6     : integer; COEFF_7  : integer; COEFF_8  : integer;
            COEFF_9     : integer; COEFF_10 : integer; COEFF_11 : integer;
            COEFF_12    : integer; COEFF_13 : integer; COEFF_14 : integer;
            COEFF_15    : integer
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            in_valid  : in  std_logic;
            in_i      : in  signed(DATA_WIDTH - 1 downto 0);
            in_q      : in  signed(DATA_WIDTH - 1 downto 0);
            out_valid : out std_logic;
            out_i     : out signed(DATA_WIDTH - 1 downto 0);
            out_q     : out signed(DATA_WIDTH - 1 downto 0)
        );
    end component;

    component pn_generator is
        generic (
            PN_LENGTH  : positive;
            POLYNOMIAL : std_logic_vector(30 downto 0);
            INIT_STATE : std_logic_vector(30 downto 0)
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            chip_en   : in  std_logic;
            load      : in  std_logic;
            seed      : in  std_logic_vector(PN_LENGTH - 1 downto 0);
            pn_chip   : out std_logic;
            pn_signed : out signed(1 downto 0)
        );
    end component;

    component despreader is
        generic (
            DATA_WIDTH : positive;
            PN_LENGTH  : positive
        );
        port (
            clk          : in  std_logic;
            rst          : in  std_logic;
            in_valid     : in  std_logic;
            in_i         : in  signed(DATA_WIDTH - 1 downto 0);
            in_q         : in  signed(DATA_WIDTH - 1 downto 0);
            pn_signed    : in  signed(1 downto 0);
            symbol_valid : out std_logic;
            symbol_i     : out signed(DATA_WIDTH - 1 downto 0);
            symbol_q     : out signed(DATA_WIDTH - 1 downto 0)
        );
    end component;

    component fft_acquisition is
        generic (
            DATA_WIDTH : positive;
            FFT_SIZE   : positive;
            LOG2N      : positive
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            in_valid  : in  std_logic;
            in_i      : in  signed(DATA_WIDTH - 1 downto 0);
            in_q      : in  signed(DATA_WIDTH - 1 downto 0);
            out_valid : out std_logic;
            peak_bin  : out unsigned(LOG2N - 1 downto 0);
            mag_peak  : out unsigned(DATA_WIDTH - 1 downto 0);
            mag_left  : out unsigned(DATA_WIDTH - 1 downto 0);
            mag_right : out unsigned(DATA_WIDTH - 1 downto 0)
        );
    end component;

    component frequency_estimator_3point is
        generic (
            MAG_WIDTH : positive;
            FFT_SIZE  : positive;
            LOG2N     : positive;
            FRAC_BITS : positive;
            DIV_WIDTH : positive
        );
        port (
            clk         : in  std_logic;
            rst         : in  std_logic;
            in_valid    : in  std_logic;
            peak_bin    : in  unsigned(LOG2N - 1 downto 0);
            mag_peak    : in  unsigned(MAG_WIDTH - 1 downto 0);
            mag_left    : in  unsigned(MAG_WIDTH - 1 downto 0);
            mag_right   : in  unsigned(MAG_WIDTH - 1 downto 0);
            out_valid   : out std_logic;
            freq_offset : out signed(LOG2N + 8 downto 0)
        );
    end component;

    component nco_frequency_corrector is
        generic (
            DATA_WIDTH  : positive;
            PHASE_WIDTH : positive;
            LUT_DEPTH   : positive
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            fcw       : in  signed(PHASE_WIDTH - 1 downto 0);
            in_valid  : in  std_logic;
            in_i      : in  signed(DATA_WIDTH - 1 downto 0);
            in_q      : in  signed(DATA_WIDTH - 1 downto 0);
            out_valid : out std_logic;
            out_i     : out signed(DATA_WIDTH - 1 downto 0);
            out_q     : out signed(DATA_WIDTH - 1 downto 0)
        );
    end component;

    component carrier_recovery_costas is
        generic (
            DATA_WIDTH  : positive;
            PHASE_WIDTH : positive;
            LUT_DEPTH   : positive;
            KP          : integer;
            KI          : integer;
            GAIN_SHIFT  : positive;
            LOCK_THRESH : positive;
            LOCK_COUNT  : positive
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            in_valid  : in  std_logic;
            in_i      : in  signed(DATA_WIDTH - 1 downto 0);
            in_q      : in  signed(DATA_WIDTH - 1 downto 0);
            out_valid : out std_logic;
            out_i     : out signed(DATA_WIDTH - 1 downto 0);
            out_q     : out signed(DATA_WIDTH - 1 downto 0);
            locked    : out std_logic
        );
    end component;

    component bpsk_symbol_detector is
        generic (
            DATA_WIDTH         : positive;
            SAMPLES_PER_SYMBOL : positive
        );
        port (
            clk        : in  std_logic;
            rst        : in  std_logic;
            in_valid   : in  std_logic;
            in_i       : in  signed(DATA_WIDTH - 1 downto 0);
            in_q       : in  signed(DATA_WIDTH - 1 downto 0);
            data_valid : out std_logic;
            rx_bit     : out std_logic;
            dbg_i      : out signed(DATA_WIDTH - 1 downto 0)
        );
    end component;

    -- =========================================================================
    -- Internal interconnect signals
    -- =========================================================================

    -- IQ interface -> matched filter
    signal iq_m_valid  : std_logic;
    signal iq_m_ready  : std_logic;
    signal iq_m_i      : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal iq_m_q      : std_logic_vector(DATA_WIDTH - 1 downto 0);

    -- Matched filter output (at chip rate)
    signal mf_valid    : std_logic;
    signal mf_i        : signed(DATA_WIDTH - 1 downto 0);
    signal mf_q        : signed(DATA_WIDTH - 1 downto 0);

    -- PN generator
    signal pn_chip     : std_logic;
    signal pn_signed   : signed(1 downto 0);

    -- FFT -> freq estimator
    signal fft_valid   : std_logic;
    signal fft_peak    : unsigned(LOG2_FFT - 1 downto 0);
    signal fft_mag_pk  : unsigned(DATA_WIDTH - 1 downto 0);
    signal fft_mag_l   : unsigned(DATA_WIDTH - 1 downto 0);
    signal fft_mag_r   : unsigned(DATA_WIDTH - 1 downto 0);

    -- Freq estimator output
    signal fest_valid  : std_logic;
    signal freq_est    : signed(LOG2_FFT + 8 downto 0);

    -- FCW register (latched from freq estimator, held constant until next update)
    signal fcw_reg     : signed(31 downto 0) := (others => '0');

    -- NCO -> Costas loop (at chip rate)
    signal nco_valid   : std_logic;
    signal nco_i       : signed(DATA_WIDTH - 1 downto 0);
    signal nco_q       : signed(DATA_WIDTH - 1 downto 0);

    -- Costas loop output (at chip rate, carrier-corrected)
    signal cos_valid   : std_logic;
    signal cos_i       : signed(DATA_WIDTH - 1 downto 0);
    signal cos_q       : signed(DATA_WIDTH - 1 downto 0);
    signal cos_locked  : std_logic;

    -- Despreader output (at symbol rate = chip rate / PN_LENGTH)
    signal desp_valid  : std_logic;
    signal desp_i      : signed(DATA_WIDTH - 1 downto 0);
    signal desp_q      : signed(DATA_WIDTH - 1 downto 0);

begin

    -- =========================================================================
    -- IQ Input Interface
    -- =========================================================================
    u_iq : iq_input_interface
        generic map (DATA_WIDTH => DATA_WIDTH)
        port map (
            clk      => clk,
            rst      => rst,
            s_valid  => s_valid,
            s_ready  => s_ready,
            s_i_data => s_i_data,
            s_q_data => s_q_data,
            m_valid  => iq_m_valid,
            m_ready  => iq_m_ready,
            m_i_data => iq_m_i,
            m_q_data => iq_m_q
        );

    -- Matched filter always ready
    iq_m_ready <= '1';

    -- =========================================================================
    -- Matched Filter
    -- =========================================================================
    u_mf : matched_filter
        generic map (
            DATA_WIDTH  => DATA_WIDTH,
            COEFF_WIDTH => 16,
            NUM_TAPS    => NUM_TAPS,
            COEFF_0  => 0,     COEFF_1  => -160,  COEFF_2  => -213,
            COEFF_3  => 256,   COEFF_4  => 1108,  COEFF_5  => 2030,
            COEFF_6  => 2561,  COEFF_7  => 2561,  COEFF_8  => 2030,
            COEFF_9  => 1108,  COEFF_10 => 256,   COEFF_11 => -213,
            COEFF_12 => -160,  COEFF_13 => 0,     COEFF_14 => 0,
            COEFF_15 => 0
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_valid  => iq_m_valid,
            in_i      => signed(iq_m_i),
            in_q      => signed(iq_m_q),
            out_valid => mf_valid,
            out_i     => mf_i,
            out_q     => mf_q
        );

    -- =========================================================================
    -- =========================================================================
    -- PN Generator:
    --   chip_en is driven by NCO corrector output valid (chip rate).
    --   The PN advances one chip per NCO output sample.
    -- =========================================================================
    u_pn : pn_generator
        generic map (
            PN_LENGTH  => PN_LENGTH,
            POLYNOMIAL => PN_POLYNOMIAL,
            INIT_STATE => "0000000000000000000000000000001"
        )
        port map (
            clk       => clk,
            rst       => rst,
            chip_en   => cos_valid,  -- advance PN at Costas output (chip) rate
            load      => '0',
            seed      => (others => '0'),
            pn_chip   => pn_chip,
            pn_signed => pn_signed
        );

    -- =========================================================================
    -- Despreader:
    --   Input: Costas loop output (carrier-corrected, chip-rate samples).
    --   Integrates over PN_LENGTH chips and outputs one despread symbol.
    --   Output is at symbol rate = chip_rate / PN_LENGTH.
    -- =========================================================================
    u_desp : despreader
        generic map (
            DATA_WIDTH => DATA_WIDTH,
            PN_LENGTH  => PN_LENGTH
        )
        port map (
            clk          => clk,
            rst          => rst,
            in_valid     => cos_valid,   -- carrier-corrected chip-rate input
            in_i         => cos_i,
            in_q         => cos_q,
            pn_signed    => pn_signed,
            symbol_valid => desp_valid,
            symbol_i     => desp_i,
            symbol_q     => desp_q
        );

    -- =========================================================================
    -- FFT Acquisition:
    --   Input: despread symbols at symbol rate.
    --   Computes FFT over FFT_SIZE symbols to estimate residual frequency offset.
    -- =========================================================================
    u_fft : fft_acquisition
        generic map (
            DATA_WIDTH => DATA_WIDTH,
            FFT_SIZE   => FFT_SIZE,
            LOG2N      => LOG2_FFT
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_valid  => desp_valid,
            in_i      => desp_i,
            in_q      => desp_q,
            out_valid => fft_valid,
            peak_bin  => fft_peak,
            mag_peak  => fft_mag_pk,
            mag_left  => fft_mag_l,
            mag_right => fft_mag_r
        );

    -- =========================================================================
    -- 3-Point Frequency Estimator
    -- =========================================================================
    u_fest : frequency_estimator_3point
        generic map (
            MAG_WIDTH => DATA_WIDTH,
            FFT_SIZE  => FFT_SIZE,
            LOG2N     => LOG2_FFT,
            FRAC_BITS => 8,
            DIV_WIDTH => 24
        )
        port map (
            clk         => clk,
            rst         => rst,
            in_valid    => fft_valid,
            peak_bin    => fft_peak,
            mag_peak    => fft_mag_pk,
            mag_left    => fft_mag_l,
            mag_right   => fft_mag_r,
            out_valid   => fest_valid,
            freq_offset => freq_est
        );

    -- Latch FCW when frequency estimator produces a new estimate.
    -- freq_est is in units of (FFT_bins × 2^FRAC_BITS).
    -- Convert to 32-bit NCO FCW:
    --   FCW = freq_est × 2^(32 - LOG2_FFT - FRAC_BITS)
    --       = freq_est << (32 - LOG2_FFT - 8)
    -- Resize to 32 bits FIRST so shift_left does not lose bits.
    proc_fcw : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                fcw_reg <= (others => '0');
            elsif fest_valid = '1' then
                fcw_reg <= shift_left(resize(freq_est, 32), 32 - LOG2_FFT - 8);
            end if;
        end if;
    end process proc_fcw;

    -- =========================================================================
    -- NCO Frequency Corrector:
    --   Applies coarse CFO correction at chip rate (matched filter output).
    --   FCW from frequency estimator; updated once per FFT block.
    -- =========================================================================
    u_nco : nco_frequency_corrector
        generic map (
            DATA_WIDTH  => DATA_WIDTH,
            PHASE_WIDTH => 32,
            LUT_DEPTH   => LUT_DEPTH
        )
        port map (
            clk       => clk,
            rst       => rst,
            fcw       => fcw_reg,
            in_valid  => mf_valid,
            in_i      => mf_i,
            in_q      => mf_q,
            out_valid => nco_valid,
            out_i     => nco_i,
            out_q     => nco_q
        );

    -- =========================================================================
    -- Costas Loop:
    --   Fine carrier phase/frequency tracking at chip rate.
    --   Input: NCO-corrected samples.
    --   Output: phase-corrected chip-rate samples.
    -- =========================================================================
    u_costas : carrier_recovery_costas
        generic map (
            DATA_WIDTH  => DATA_WIDTH,
            PHASE_WIDTH => 24,
            LUT_DEPTH   => LUT_DEPTH,
            KP          => COSTAS_KP,
            KI          => COSTAS_KI,
            GAIN_SHIFT  => COSTAS_GAIN_SHIFT,
            LOCK_THRESH => 64,
            LOCK_COUNT  => 1024
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_valid  => nco_valid,
            in_i      => nco_i,
            in_q      => nco_q,
            out_valid => cos_valid,
            out_i     => cos_i,
            out_q     => cos_q,
            locked    => cos_locked
        );

    -- =========================================================================
    -- BPSK Symbol Detector:
    --   Input: despread symbols at symbol rate (from despreader).
    --   SAMPLES_PER_SYMBOL=1 because the despreader already decimates to
    --   symbol rate; each despread output is one complete symbol.
    -- =========================================================================
    u_det : bpsk_symbol_detector
        generic map (
            DATA_WIDTH         => DATA_WIDTH,
            SAMPLES_PER_SYMBOL => 1   -- despreader already at symbol rate
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_valid   => desp_valid,
            in_i       => desp_i,
            in_q       => desp_q,
            data_valid => data_valid,
            rx_bit     => rx_bit,
            dbg_i      => open
        );

    locked <= cos_locked;

    -- =========================================================================
    -- Debug outputs
    -- =========================================================================
    dbg_mf_valid  <= mf_valid;
    dbg_mf_i      <= mf_i;
    dbg_mf_q      <= mf_q;
    dbg_fft_valid <= fft_valid;
    dbg_peak_bin  <= fft_peak;
    dbg_freq_est  <= freq_est;
    dbg_costas_i  <= cos_i;
    dbg_costas_q  <= cos_q;

end architecture rtl;
