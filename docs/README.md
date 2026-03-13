# BPSK Spread-Spectrum Receiver — Design Documentation

## 1. Block Diagram

```
                     ┌───────────────────────────────────────────────────────────────────┐
                     │                  BPSK Spread-Spectrum Receiver                     │
                     │                                                                   │
 I/Q Input           │  ┌──────────────┐   ┌────────────────┐   ┌────────────────────┐ │
 (16-bit signed) ────┼─►│  IQ Input    │──►│  Matched       │──►│  PN Generator      │ │
  AXI-Stream         │  │  Interface   │   │  Filter (FIR)  │   │  (LFSR)            │ │
                     │  └──────────────┘   └────────────────┘   └────────┬───────────┘ │
                     │                                                     │ pn_signed   │
                     │                                           ┌─────────▼───────────┐ │
                     │                                           │  Despreader         │ │
                     │                                           │  (integrate+dump)   │ │
                     │                                           └─────────┬───────────┘ │
                     │                                                     │ despread sym │
                     │  ┌──────────────────────────────┐   ┌──────────────▼───────────┐ │
                     │  │  3-Point Frequency Estimator │◄──│  FFT Acquisition         │ │
                     │  │  (parabolic interpolation)   │   │  (Radix-2 DIT)           │ │
                     │  └──────────────┬───────────────┘   └──────────────────────────┘ │
                     │                 │ freq_offset                                     │
                     │  ┌──────────────▼───────────────┐                                │
                     │  │  NCO Frequency Corrector      │◄── (matched filter output)     │
                     │  │  (phase accumulator + mixer)  │                                │
                     │  └──────────────┬───────────────┘                                │
                     │                 │ CFO-corrected I/Q                               │
                     │  ┌──────────────▼───────────────┐                                │
                     │  │  Costas Loop                  │                                │
                     │  │  (PI filter + NCO)            │                                │
                     │  └──────────────┬───────────────┘                                │
                     │                 │ phase-corrected I/Q                             │
                     │  ┌──────────────▼───────────────┐                                │
 rx_bit ◄────────────┼──│  BPSK Symbol Detector        │                                │
 data_valid          │  │  (sign of I channel)         │                                │
                     │  └──────────────────────────────┘                                │
                     └───────────────────────────────────────────────────────────────────┘
```

---

## 2. Module Descriptions

### `iq_input_interface.vhd`

**Purpose:** AXI-Stream compatible input interface that registers incoming I/Q samples.

**Key design decisions:**
- Implements a one-element skid buffer to decouple upstream and downstream handshakes.
- `s_ready` is deasserted when the buffer is full *and* the downstream is not consuming.
- Data is registered on the rising clock edge when both `s_valid` and `s_ready` are asserted.

**Interface signals:**

| Signal | Direction | Width | Description |
|--------|-----------|-------|-------------|
| `s_valid` | in | 1 | Upstream data valid |
| `s_ready` | out | 1 | Upstream ready (backpressure) |
| `s_i_data` | in | `DATA_WIDTH` | In-phase sample |
| `s_q_data` | in | `DATA_WIDTH` | Quadrature sample |
| `m_valid` | out | 1 | Downstream data valid |
| `m_ready` | in | 1 | Downstream ready |
| `m_i_data` | out | `DATA_WIDTH` | Registered I sample |
| `m_q_data` | out | `DATA_WIDTH` | Registered Q sample |

---

### `matched_filter.vhd`

**Purpose:** FIR filter matched to the transmitter pulse shape (default: RRC-like symmetric 16-tap filter).

**Key design decisions:**
- Shift-register architecture with a direct-form MAC loop.
- All 16 tap coefficients are passed as integer generics so they can be overridden without re-editing the file.
- Accumulator width = `DATA_WIDTH + COEFF_WIDTH + 5` to accommodate 32 taps without overflow.
- Output is truncated (arithmetic right-shift by `COEFF_WIDTH-1`) back to `DATA_WIDTH`.

**Bit-growth:** Input 16-bit × Coefficient 16-bit = 32-bit product. After accumulation over 16 taps (+4 bits): 36-bit ACC. Truncated to 16-bit output by dropping 19 LSBs.

---

### `pn_generator.vhd`

**Purpose:** LFSR-based maximal-length PN sequence generator.

**Key design decisions:**
- Configurable `PN_LENGTH` (1–31) and feedback `POLYNOMIAL` via generics.
- Synchronous state load via `load`/`seed` ports for programmable phase offset.
- Output `pn_signed` maps chip bit 0→+1, 1→−1 as a 2-bit signed value.
- The LFSR advances only when `chip_en` is asserted, allowing rate control.

**Polynomial example:** For PN length 31 the polynomial x³¹ + x³ + 1 gives a period of 2³¹−1 = 2,147,483,647 chips.

---

### `despreader.vhd`

**Purpose:** Multiply incoming I/Q samples by PN chip (+1/−1) and integrate-and-dump over one PN period.

**Key design decisions:**
- Despreading by sign flip: chip=+1 passes sample through; chip=−1 negates sample (no multiplier needed).
- Accumulator sized to `DATA_WIDTH + ACC_EXTRA` (where `ACC_EXTRA=6`) to hold up to 64 chips without overflow.
- At the end of each PN period, the accumulated value is right-shifted by `ACC_EXTRA` (equivalent to dividing by ~64) and output as a despread symbol.

---

### `fft_acquisition.vhd`

**Purpose:** Radix-2 DIT FFT for coarse frequency acquisition.

**Key design decisions:**
- Three-state FSM: FILL → COMPUTE → PEAK.
- Bit-reversal is applied at input time during FILL state.
- Twiddle factors are ROM constants (generated by `impure function` at elaboration time from `ieee.math_real`).
- Each butterfly divides both outputs by 2 (scaled FFT), preventing overflow through all `log2(N)` stages.
- Magnitude estimate: `|I| + |Q|` (fast Manhattan approximation). Equivalent to ≈1.08× true magnitude for uniformly distributed angles.
- Peak detection: a linear scan after all butterfly stages.

**Latency:** N (fill) + N/2 × log₂N (butterflies) + N (peak scan) clock cycles per output.

---

### `frequency_estimator_3point.vhd`

**Purpose:** Sub-bin frequency estimation using 3-point parabolic interpolation on FFT magnitudes.

**Formula:**
```
delta = 0.5 × (M[k+1] − M[k−1]) / (2×M[k] − M[k−1] − M[k+1])
freq_offset = (k + delta) × fs / N
```

**Key design decisions:**
- Stage 1 computes numerator and denominator from the three magnitude inputs.
- An iterative non-restoring binary divider (one bit per cycle) computes `|numerator| × 2^FRAC_BITS / denominator` to `DIV_WIDTH` bits of precision.
- Sign bit of numerator is stored and applied to the quotient after division.
- Division-by-zero is protected by forcing denominator to 1 when it would be zero.
- Output `freq_offset` is signed fixed-point with `FRAC_BITS` fractional bits.

---

### `nco_frequency_corrector.vhd`

**Purpose:** Apply coarse carrier frequency offset (CFO) correction using an NCO.

**Key design decisions:**
- 32-bit phase accumulator advances by `FCW` (frequency control word) each sample.
- Sine/cosine are derived from a quarter-wave LUT (256 entries) with symmetry logic for all four quadrants, reducing ROM to 1/4 of full circle.
- Three-pipeline-stage complex rotation:
  - Stage 1: register inputs + LUT lookup
  - Stage 2: four multiplications (I×cos, Q×sin, Q×cos, I×sin)
  - Stage 3: addition/subtraction + truncation

**FCW calculation:** `FCW = round(f_offset / f_s × 2^32)`

---

### `carrier_recovery_costas.vhd`

**Purpose:** Fine phase tracking using a second-order Costas loop.

**Key design decisions:**
- Phase detector: `error = I × sign(Q)` (decision-directed, suitable for BPSK where Q should be zero after lock).
- PI loop filter: `phase_correction = Kp × error + Ki × ∫error dt`
- Configurable `Kp`, `Ki`, and `GAIN_SHIFT` generics control loop bandwidth.
- Lock detector: counts cycles where `|error| < LOCK_THRESH`; asserts `locked` after `LOCK_COUNT` consecutive samples below threshold.
- Same quarter-wave LUT approach as the NCO corrector (separate instance, independent phase accumulator).

---

### `bpsk_symbol_detector.vhd`

**Purpose:** Hard decision on the I channel to recover the BPSK bit stream.

**Key design decisions:**
- Sign of `in_i` determines bit: I ≥ 0 → bit=1, I < 0 → bit=0.
- A sample counter selects the optimal sampling instant at `SAMPLES_PER_SYMBOL/2` (midpoint of the symbol period).
- `data_valid` strobe pulses once per recovered symbol.
- Debug port `dbg_i` captures the raw I value at the decision point.

---

### `top_level_receiver.vhd`

**Purpose:** Interconnect wrapper instantiating all modules.

**Key design decisions:**
- Matched filter output feeds *both* the despreader/FFT chain *and* the NCO frequency corrector simultaneously, so coarse and fine correction operate on the same filtered signal.
- FCW register latches the frequency estimate from the 3-point estimator and holds it steady between FFT blocks.
- All modules share a single clock domain.
- Debug ports pass through key intermediate signals for waveform inspection.

---

## 3. Timing Assumptions

| Module | Latency (cycles) | Notes |
|--------|-----------------|-------|
| IQ Input Interface | 1 | Registered output |
| Matched Filter | 2 | FIR delay + output register |
| PN Generator | 0 | Combinatorial output registered externally |
| Despreader | PN_LENGTH | Integrate-and-dump period |
| FFT Acquisition | N + N/2·log₂N + N | Fill + compute + peak scan |
| Freq Estimator | DIV_WIDTH + 3 | Iterative divider |
| NCO Corrector | 3 | Three pipeline stages |
| Costas Loop | 4 | LUT + three pipeline stages + loop update |
| Symbol Detector | SAMPLES_PER_SYMBOL/2 | Midpoint sampling |

**Total acquisition latency:** Dominated by the FFT block. For FFT_SIZE=64: 64 + 192 + 64 = 320 cycles per FFT block, plus `PN_LENGTH × SAMPLES_PER_SYMBOL` = 31×4 = 124 cycles to fill one block.

**Clock domain:** All modules operate in a single synchronous clock domain. No CDC crossings.

---

## 4. Fixed-Point Scaling

### Bit Growth Through the Chain

| Stage | Width | Notes |
|-------|-------|-------|
| Input I/Q | 16-bit signed | Raw ADC samples |
| After Matched Filter | 16-bit signed | Truncated from 37-bit accumulator |
| After Despreader | 16-bit signed | Truncated from 22-bit accumulator |
| FFT Internal | 22-bit signed | 16 + log₂(64) = 22 bits |
| FFT Output (magnitude) | 16-bit unsigned | Truncated for peak output |
| After NCO Corrector | 16-bit signed | Truncated from 32-bit product |
| After Costas Loop | 16-bit signed | Truncated from 32-bit product |

### Rounding Strategy
- All truncations use **arithmetic right-shift** (floor rounding for positive numbers, ceiling for negative), which is unbiased for symmetric signals.
- No saturation logic is included by default; it can be added if overflow is detected in simulation.

### LUT Precision
- Quarter-wave LUTs use 16-bit entries scaled by `2^15 - 1 = 32767`.
- After complex multiplication and right-shift by 15, the output returns to 16-bit range.

---

## 5. Simulation Instructions (ModelSim)

### Prerequisites
- ModelSim DE or ModelSim Intel FPGA Edition
- VHDL-2008 support

### Compile

```tcl
# From the project root directory
vlib work
vmap work work

# Compile all source files
vcom -2008 -work work \
    src/iq_input_interface.vhd \
    src/matched_filter.vhd \
    src/pn_generator.vhd \
    src/despreader.vhd \
    src/fft_acquisition.vhd \
    src/frequency_estimator_3point.vhd \
    src/nco_frequency_corrector.vhd \
    src/carrier_recovery_costas.vhd \
    src/bpsk_symbol_detector.vhd \
    src/top_level_receiver.vhd \
    sim/tb_bpsk_receiver.vhd
```

### Simulate

```tcl
vsim -t 1ns work.tb_bpsk_receiver

# Add all top-level signals to waveform
add wave -divider "Input"
add wave /tb_bpsk_receiver/clk
add wave /tb_bpsk_receiver/rst
add wave /tb_bpsk_receiver/s_valid
add wave -radix decimal /tb_bpsk_receiver/s_i_data
add wave -radix decimal /tb_bpsk_receiver/s_q_data

add wave -divider "Matched Filter (Debug)"
add wave /tb_bpsk_receiver/dbg_mf_valid
add wave -radix decimal /tb_bpsk_receiver/dbg_mf_i
add wave -radix decimal /tb_bpsk_receiver/dbg_mf_q

add wave -divider "FFT Acquisition"
add wave /tb_bpsk_receiver/dbg_fft_valid
add wave -radix unsigned /tb_bpsk_receiver/dbg_peak_bin

add wave -divider "Frequency Estimate"
add wave -radix decimal /tb_bpsk_receiver/dbg_freq_est

add wave -divider "Costas Loop"
add wave -radix decimal /tb_bpsk_receiver/dbg_costas_i
add wave -radix decimal /tb_bpsk_receiver/dbg_costas_q
add wave /tb_bpsk_receiver/locked

add wave -divider "Output"
add wave /tb_bpsk_receiver/data_valid
add wave /tb_bpsk_receiver/rx_bit

run -all
```

### Expected Output

The simulator console should show:
```
# === BPSK Receiver Testbench Results ===
#   Symbols transmitted : 256
#   Symbols checked     : 252
#   Bit errors          : <N>
#   SNR (dB)            : 20.0
#   CFO (norm)          : 5.0e-3
# PASS: Bit error rate within acceptable range.
```

---

## 6. Synthesis Notes

### Target Platform
- **Primary:** Xilinx 7-series (Artix-7, Kintex-7) or UltraScale+
- **Compatibility:** Any FPGA with 18K×2 block RAMs and DSP48 slices

### Resource Estimates (FFT_SIZE=64, DATA_WIDTH=16, PN_LENGTH=31)

| Module | LUTs | FFs | DSP48 | BRAM |
|--------|------|-----|-------|------|
| IQ Input Interface | ~30 | ~35 | 0 | 0 |
| Matched Filter (16 taps) | ~200 | ~250 | 4–8 | 0 |
| PN Generator | ~50 | ~35 | 0 | 0 |
| Despreader | ~100 | ~60 | 0 | 0 |
| FFT Acquisition (N=64) | ~800 | ~600 | 8 | 0–1 |
| Freq Estimator | ~300 | ~200 | 2 | 0 |
| NCO Corrector | ~400 | ~350 | 4 | 0–1 |
| Costas Loop | ~450 | ~400 | 4 | 0–1 |
| Symbol Detector | ~40 | ~30 | 0 | 0 |
| **Total (approx.)** | **~2400** | **~1960** | **~22** | **~3** |

### Timing
- Design targets **100 MHz** on Artix-7 (-1 speed grade).
- Critical paths are in the FFT butterfly multipliers and the NCO/Costas LUT+multiplier chains.
- Retiming can be enabled in Vivado synthesis (`retiming` strategy) if timing closure is difficult.

### Vendor Primitives
- No vendor-specific primitives are instantiated; all logic is described behaviourally.
- Vivado / Quartus will infer DSP48/DSP slices for all multiply-accumulate operations automatically.
- The twiddle-factor and LUT ROMs will be inferred as distributed LUT-RAM or BRAM depending on the synthesis tool's decision tree.

---

## 7. Parameter Reference

| Generic | Default | Description |
|---------|---------|-------------|
| `DATA_WIDTH` | 16 | I/Q sample bit width |
| `FFT_SIZE` | 64 | FFT block size (power of 2) |
| `LOG2_FFT` | 6 | log₂(FFT_SIZE) |
| `PN_LENGTH` | 31 | PN sequence length in chips |
| `PN_POLYNOMIAL` | x³¹+x³+1 | LFSR feedback polynomial |
| `NUM_TAPS` | 16 | Matched filter tap count |
| `SAMPLES_PER_SYMBOL` | 4 | Oversampling ratio |
| `COSTAS_KP` | 128 | Costas loop proportional gain |
| `COSTAS_KI` | 4 | Costas loop integral gain |
| `COSTAS_GAIN_SHIFT` | 12 | Right-shift applied to loop output |
| `LUT_DEPTH` | 256 | Quarter-wave LUT entries |

---

## 8. Project Structure

```
bpsk-receiver/
├── src/
│   ├── iq_input_interface.vhd         IQ streaming input
│   ├── matched_filter.vhd             Parameterizable FIR filter
│   ├── pn_generator.vhd               LFSR PN sequence generator
│   ├── despreader.vhd                 PN despreading + integrate-and-dump
│   ├── fft_acquisition.vhd            Radix-2 DIT FFT acquisition
│   ├── frequency_estimator_3point.vhd 3-point parabolic freq estimator
│   ├── nco_frequency_corrector.vhd    NCO + complex frequency corrector
│   ├── carrier_recovery_costas.vhd    Costas loop carrier recovery
│   ├── bpsk_symbol_detector.vhd       BPSK hard-decision symbol detector
│   └── top_level_receiver.vhd         Top-level interconnect
├── sim/
│   └── tb_bpsk_receiver.vhd           Full system testbench
├── docs/
│   └── README.md                      This document
└── README.md                          Project overview
```
