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
        dbg_mf_valid   : out std_logic;
        dbg_mf_i       : out signed(DATA_WIDTH - 1 downto 0);
        dbg_mf_q       : out signed(DATA_WIDTH - 1 downto 0);
        dbg_fft_valid  : out std_logic;
        dbg_peak_bin   : out unsigned(LOG2_FFT - 1 downto 0);
        dbg_freq_est   : out signed(LOG2_FFT + 8 downto 0);  -- 8 frac bits
        dbg_costas_i   : out signed(DATA_WIDTH - 1 downto 0);
        dbg_costas_q   : out signed(DATA_WIDTH - 1 downto 0);
        -- Despreader output: PN correlation peak observable in Vivado waveform
        dbg_desp_valid : out std_logic;
        dbg_desp_i     : out signed(DATA_WIDTH - 1 downto 0);
        dbg_desp_q     : out signed(DATA_WIDTH - 1 downto 0)
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

    -- NCO -> despreader (at chip rate, before Costas)
    signal nco_valid   : std_logic;
    signal nco_i       : signed(DATA_WIDTH - 1 downto 0);
    signal nco_q       : signed(DATA_WIDTH - 1 downto 0);

    -- Costas loop output (at symbol rate, after despreader)
    signal cos_valid   : std_logic;
    signal cos_i       : signed(DATA_WIDTH - 1 downto 0);
    signal cos_q       : signed(DATA_WIDTH - 1 downto 0);
    signal cos_locked  : std_logic;

    -- Despreader output (at symbol rate = chip rate / PN_LENGTH)
    signal desp_valid  : std_logic;
    signal desp_i      : signed(DATA_WIDTH - 1 downto 0);
    signal desp_q      : signed(DATA_WIDTH - 1 downto 0);

    -- Complex-squaring stage: (I+jQ)^2 = (I^2-Q^2) + j*2IQ
    -- Removes BPSK data modulation (d^2=1) so the FFT sees a clean tone at 2*f_CFO.
    signal sq_valid    : std_logic := '0';
    signal sq_i        : signed(DATA_WIDTH - 1 downto 0) := (others => '0');
    signal sq_q        : signed(DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Matched-filter timing alignment:
    -- The MF's out_valid (driven by valid_d1) fires one clock before out_i/out_q
    -- contain the correct accumulated result for the current chip.  A 1-cycle
    -- delay register aligns the valid strobe with the data before the NCO sees it,
    -- preventing a systematic PN misalignment (+1 chip offset) in the despreader.
    signal mf_valid_aligned : std_logic := '0';

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
            -- Single-tap pass-through for 1-sample-per-chip operation.
            -- The RRC coefficients (COEFF_0=0 at the main lag) cause the PN
            -- autocorrelation sum to use only the -1/31 side-lobes, collapsing
            -- the despreader output to ~44 LSB -- too small for the squaring
            -- stage.  A single tap at tap-0 (COEFF_0=32767) preserves the full
            -- chip amplitude while the MAC timing fix ensures chip[k] maps to
            -- tap-0 at the correct clock cycle.
            COEFF_0  => 32767, COEFF_1  => 0,     COEFF_2  => 0,
            COEFF_3  => 0,     COEFF_4  => 0,     COEFF_5  => 0,
            COEFF_6  => 0,     COEFF_7  => 0,     COEFF_8  => 0,
            COEFF_9  => 0,     COEFF_10 => 0,     COEFF_11 => 0,
            COEFF_12 => 0,     COEFF_13 => 0,     COEFF_14 => 0,
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

    -- Delay mf_valid by one cycle so it coincides with the correct out_i/out_q data.
    -- (The MF's valid_d1 fires one cycle before proc_out has latched the current
    -- chip's MAC result; the extra register stage removes that skew and prevents
    -- a +1 chip PN misalignment in the despreader.)
    proc_mf_align : process(clk) is
    begin
        if rising_edge(clk) then
            if rst = '1' then
                mf_valid_aligned <= '0';
            else
                mf_valid_aligned <= mf_valid;
            end if;
        end if;
    end process proc_mf_align;

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
            chip_en   => nco_valid,  -- advance PN at NCO output (chip) rate
            load      => '0',
            seed      => (others => '0'),
            pn_chip   => pn_chip,
            pn_signed => pn_signed
        );

    -- =========================================================================
    -- Despreader:
    --   Input: NCO-corrected chip-rate samples (carrier frequency corrected).
    --   Integrates over PN_LENGTH chips and outputs one despread symbol.
    --   Output is at symbol rate = chip_rate / PN_LENGTH.
    --
    --   The Costas loop now operates AFTER the despreader (at symbol rate),
    --   so the despreader uses the raw NCO output.  This decouples coarse
    --   frequency correction (FFT/NCO) from fine phase tracking (Costas).
    -- =========================================================================
    u_desp : despreader
        generic map (
            DATA_WIDTH => DATA_WIDTH,
            PN_LENGTH  => PN_LENGTH
        )
        port map (
            clk          => clk,
            rst          => rst,
            in_valid     => nco_valid,   -- NCO-corrected chip-rate input
            in_i         => nco_i,
            in_q         => nco_q,
            pn_signed    => pn_signed,
            symbol_valid => desp_valid,
            symbol_i     => desp_i,
            symbol_q     => desp_q
        );

    -- =========================================================================
    -- Complex Squaring Stage:
    --   Input: despread BPSK symbols d_m * e^(j*phi_m).
    --   Output: squared symbols e^(j*2*phi_m).
    --
    --   Squaring removes the BPSK data modulation (d_m^2 = 1), leaving a pure
    --   complex exponential at twice the residual carrier frequency offset.
    --   Without this, the FFT sees random-polarity phasors and cannot detect
    --   the carrier frequency offset peak (E[d_m]=0 → E[FFT output]=0).
    --
    --   (I + jQ)^2 = (I^2 - Q^2) + j * 2*I*Q
    --   Scaled by 2^(-DATA_WIDTH) to keep result within DATA_WIDTH bits.
    -- =========================================================================
    proc_square : process(clk) is
        variable prod_ii : signed(2 * DATA_WIDTH - 1 downto 0);
        variable prod_qq : signed(2 * DATA_WIDTH - 1 downto 0);
        variable prod_iq : signed(2 * DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                sq_valid <= '0';
                sq_i     <= (others => '0');
                sq_q     <= (others => '0');
            else
                sq_valid <= desp_valid;
                if desp_valid = '1' then
                    prod_ii := desp_i * desp_i;
                    prod_qq := desp_q * desp_q;
                    prod_iq := desp_i * desp_q;
                    -- Scale down by 2^DATA_WIDTH to keep within DATA_WIDTH bits.
                    -- Real part: (I^2 - Q^2) / 2^DATA_WIDTH
                    sq_i <= resize(shift_right(prod_ii - prod_qq, DATA_WIDTH),
                                   DATA_WIDTH);
                    -- Imag part: 2*I*Q / 2^DATA_WIDTH = I*Q / 2^(DATA_WIDTH-1)
                    sq_q <= resize(shift_right(prod_iq, DATA_WIDTH - 1),
                                   DATA_WIDTH);
                end if;
            end if;
        end if;
    end process proc_square;

    -- =========================================================================
    -- FFT Acquisition:
    --   Input: SQUARED despread symbols (data modulation removed).
    --   Computes FFT over FFT_SIZE symbols; peak bin indicates 2*f_CFO.
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
            in_valid  => sq_valid,   -- squared despread symbols (data removed)
            in_i      => sq_i,
            in_q      => sq_q,
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
    -- freq_est is peak_bin * 2^FRAC_BITS (FRAC_BITS=8), where peak_bin is the
    -- FFT bin of the SQUARED despread signal tone (at 2*f_CFO in the symbol domain).
    --
    -- Converting to NCO FCW (chip-rate accumulator, 32-bit full circle):
    --   f_tone   = peak_bin * symbol_rate / FFT_SIZE
    --            = peak_bin * chip_rate / (PN_LENGTH * FFT_SIZE)
    --   f_CFO    = f_tone / 2          (squaring doubled the frequency)
    --   FCW      = f_CFO / chip_rate * 2^32
    --            = peak_bin / (2 * PN_LENGTH * FFT_SIZE) * 2^32
    --            = freq_est * 2^(32 - FRAC_BITS - LOG2_FFT - 1 - log2(PN_LENGTH))
    --
    -- With FRAC_BITS=8, log2(PN_LENGTH=31) ≈ 5 (using 2^5=32 approximation):
    --   FCW = freq_est * 2^(32-8-LOG2_FFT-1-5) = freq_est * 2^(18-LOG2_FFT)
    --
    -- Negative-frequency handling: if peak_bin > FFT_SIZE/2 the true frequency
    -- is negative.  Bit (LOG2_FFT+7) of freq_est is '1' exactly when this occurs.
    -- Correct by subtracting FFT_SIZE * 2^FRAC_BITS = FFT_SIZE * 256.
    proc_fcw : process(clk) is
        variable freq_adj : signed(31 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                fcw_reg <= (others => '0');
            elsif fest_valid = '1' then
                -- Wrap upper-half bins to negative frequency
                if freq_est(LOG2_FFT + 7) = '1' then
                    freq_adj := resize(freq_est, 32) -
                                to_signed(FFT_SIZE * 256, 32);
                else
                    freq_adj := resize(freq_est, 32);
                end if;
                -- Scale to 32-bit NCO FCW.  LOG2_FFT must be <= 18.
                fcw_reg <= shift_left(freq_adj, 18 - LOG2_FFT);
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
            in_valid  => mf_valid_aligned,
            in_i      => mf_i,
            in_q      => mf_q,
            out_valid => nco_valid,
            out_i     => nco_i,
            out_q     => nco_q
        );

    -- =========================================================================
    -- Costas Loop:
    --   Fine carrier phase tracking at SYMBOL rate (after despreader).
    --   Moving the Costas to symbol rate gives 31× higher SNR per sample and
    --   fixes the gain-scaling problem: with PHASE_WIDTH=16 and GS=7 the
    --   effective loop gain K_eff = KP*A*2π/(2^GS * 2^16) ≈ 0.38 < 1
    --   (stable, ~3-symbol time constant).
    --
    --   Input:  despread symbols (symbol rate = chip_rate / PN_LENGTH).
    --   Output: phase-corrected symbols → BPSK symbol detector.
    --
    --   The despreader output BEFORE the Costas is also fed directly to the
    --   squaring/FFT pipeline for coarse frequency estimation, so the two
    --   loops (FFT/NCO coarse + Costas fine) are independent.
    -- =========================================================================
    u_costas : carrier_recovery_costas
        generic map (
            DATA_WIDTH  => DATA_WIDTH,
            PHASE_WIDTH => 16,          -- 16-bit phase acc: correct gain scaling
            LUT_DEPTH   => LUT_DEPTH,
            KP          => COSTAS_KP,
            KI          => COSTAS_KI,
            GAIN_SHIFT  => COSTAS_GAIN_SHIFT,
            LOCK_THRESH => 512,         -- ~7-sigma for symbol-rate SNR ~35 dB
            LOCK_COUNT  => 100          -- symbols (≈ 3100 chips)
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_valid  => desp_valid,    -- symbol-rate input
            in_i      => desp_i,
            in_q      => desp_q,
            out_valid => cos_valid,
            out_i     => cos_i,
            out_q     => cos_q,
            locked    => cos_locked
        );

    -- =========================================================================
    -- BPSK Symbol Detector:
    --   Input: Costas-corrected despread symbols (symbol rate).
    --   SAMPLES_PER_SYMBOL=1: despreader+Costas already at symbol rate.
    -- =========================================================================
    u_det : bpsk_symbol_detector
        generic map (
            DATA_WIDTH         => DATA_WIDTH,
            SAMPLES_PER_SYMBOL => 1   -- despreader already at symbol rate
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_valid   => cos_valid,    -- Costas-corrected symbol-rate input
            in_i       => cos_i,
            in_q       => cos_q,
            data_valid => data_valid,
            rx_bit     => rx_bit,
            dbg_i      => open
        );

    locked <= cos_locked;

    -- =========================================================================
    -- Debug outputs
    -- =========================================================================
    dbg_mf_valid   <= mf_valid_aligned;
    dbg_mf_i       <= mf_i;
    dbg_mf_q       <= mf_q;
    dbg_fft_valid  <= fft_valid;
    dbg_peak_bin   <= fft_peak;
    dbg_freq_est   <= freq_est;
    dbg_costas_i   <= cos_i;
    dbg_costas_q   <= cos_q;
    dbg_desp_valid <= desp_valid;
    dbg_desp_i     <= desp_i;
    dbg_desp_q     <= desp_q;

end architecture rtl;
