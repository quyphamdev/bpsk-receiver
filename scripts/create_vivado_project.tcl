# =============================================================================
# create_vivado_project.tcl
#
# Creates a Vivado project for the BPSK Receiver design, adds all source files
# (VHDL 2008) and the testbench, configures the synthesis and simulation tops,
# and opens the project in the Vivado GUI.
#
# Usage (from the repository root):
#   vivado -source scripts/create_vivado_project.tcl
#
# Or, to run in batch mode without opening the GUI:
#   vivado -mode batch -source scripts/create_vivado_project.tcl
# =============================================================================

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
set script_dir   [file dirname [file normalize [info script]]]
set project_root [file dirname $script_dir]

# ---------------------------------------------------------------------------
# Project settings – change FPGA_PART to match your target device
# ---------------------------------------------------------------------------
set project_name bpsk_receiver
set project_dir  [file join $project_root vivado_project]
set fpga_part    xc7a35tcpg236-1

# ---------------------------------------------------------------------------
# Create project
# ---------------------------------------------------------------------------
create_project $project_name $project_dir -part $fpga_part -force

set_property target_language    VHDL [current_project]
set_property simulator_language VHDL [current_project]

# ---------------------------------------------------------------------------
# Source files (compilation order: leaves first, top-level last)
# ---------------------------------------------------------------------------
set src_files [list \
    [file join $project_root src iq_input_interface.vhd]          \
    [file join $project_root src matched_filter.vhd]              \
    [file join $project_root src pn_generator.vhd]                \
    [file join $project_root src despreader.vhd]                  \
    [file join $project_root src fft_acquisition.vhd]             \
    [file join $project_root src frequency_estimator_3point.vhd]  \
    [file join $project_root src nco_frequency_corrector.vhd]     \
    [file join $project_root src carrier_recovery_costas.vhd]     \
    [file join $project_root src bpsk_symbol_detector.vhd]        \
    [file join $project_root src top_level_receiver.vhd]          \
]

add_files -norecurse $src_files
set_property file_type {VHDL 2008} [get_files $src_files]

# Set synthesis top
set_property top top_level_receiver [current_fileset]

# ---------------------------------------------------------------------------
# Testbench / simulation files
# ---------------------------------------------------------------------------
set sim_files [list \
    [file join $project_root sim tb_bpsk_receiver.vhd] \
]

add_files -fileset sim_1 -norecurse $sim_files
set_property file_type {VHDL 2008} [get_files -fileset sim_1 $sim_files]

# Set simulation top
set_property top         tb_bpsk_receiver [get_filesets sim_1]
set_property top_lib     xil_defaultlib   [get_filesets sim_1]

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
puts "INFO: Project '$project_name' created at $project_dir"
puts "INFO: Synthesis top  : top_level_receiver"
puts "INFO: Simulation top : tb_bpsk_receiver"
puts "INFO: Target part    : $fpga_part"
puts "INFO: To re-open later: vivado [file join $project_dir ${project_name}.xpr]"

# Open the Vivado GUI (no-op when Vivado is already running in GUI mode)
start_gui
