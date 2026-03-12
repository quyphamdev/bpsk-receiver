# BPSK Spread-Spectrum Receiver

A complete, synthesizable BPSK spread-spectrum receiver implemented in **VHDL-2008**, compatible with **ModelSim simulation** and **Xilinx FPGA** toolchains.

## Block Diagram

```
 I/Q Input    ┌──────────┐   ┌─────────┐   ┌───────────┐   ┌──────────────┐
─────────────►│ IQ Input │──►│ Matched │──►│ PN Desp-  │──►│ FFT Acqui-   │
 (16-bit      │ Interface│   │ Filter  │   │ reader    │   │ sition       │
  signed)     └──────────┘   └─────────┘   └───────────┘   └──────┬───────┘
                                                                   │
                                                  ┌────────────────┘
                                                  ▼
 Bit Output   ┌──────────┐   ┌─────────┐   ┌───────────┐   ┌──────────────┐
◄─────────────│ Symbol   │◄──│ Costas  │◄──│ NCO Freq  │◄──│ 3-Point Freq │
              │ Detector │   │ Loop    │   │ Corrector │   │ Estimator    │
              └──────────┘   └─────────┘   └───────────┘   └──────────────┘
```

## Project Structure

| Path | Description |
|------|-------------|
| `src/iq_input_interface.vhd` | AXI-Stream I/Q input with valid/ready handshake |
| `src/matched_filter.vhd` | Parameterizable FIR matched filter |
| `src/pn_generator.vhd` | LFSR-based PN sequence generator |
| `src/despreader.vhd` | PN despreading with integrate-and-dump |
| `src/fft_acquisition.vhd` | Radix-2 DIT FFT for coarse acquisition |
| `src/frequency_estimator_3point.vhd` | 3-point parabolic interpolation frequency estimator |
| `src/nco_frequency_corrector.vhd` | NCO + complex frequency corrector |
| `src/carrier_recovery_costas.vhd` | Costas loop for BPSK carrier recovery |
| `src/bpsk_symbol_detector.vhd` | BPSK hard-decision symbol detector |
| `src/top_level_receiver.vhd` | Top-level interconnect |
| `sim/tb_bpsk_receiver.vhd` | Full-system testbench |
| `docs/README.md` | Architecture documentation |

## Quick Start (ModelSim)

```tcl
vlib work
vcom -2008 src/iq_input_interface.vhd src/matched_filter.vhd \
    src/pn_generator.vhd src/despreader.vhd src/fft_acquisition.vhd \
    src/frequency_estimator_3point.vhd src/nco_frequency_corrector.vhd \
    src/carrier_recovery_costas.vhd src/bpsk_symbol_detector.vhd \
    src/top_level_receiver.vhd sim/tb_bpsk_receiver.vhd
vsim work.tb_bpsk_receiver
run -all
```

See [docs/README.md](docs/README.md) for full documentation including timing, fixed-point scaling, and synthesis notes.
