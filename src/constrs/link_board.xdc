# link_board.xdc — board link pins for the board designs (board_a_top / board_b_top), role independent;
# copied from link_pins.xdc (the link test design, hardware-verified at 100 MHz). The role-dependent clock and
# IO timing is applied after synthesis by tools/link_timing.tcl.
# Lane vector bit -> header pin:  [15:0]->5..20  [16]->21 [17]->22 [18]->23 [19]->24 [20]->25 (LCLK, MRCC)
#                                  [21]->26  [27:22]->27..32  [31:28]->37..40
# Balls from the Puzhi PA-Starlite user manual §3.15 (JM1 = bank 16, JM2 = bank 15, both 3.3 V).
set_property PACKAGE_PIN E16 [get_ports {jm1[0]}];      set_property PACKAGE_PIN G17 [get_ports {jm2[0]}]   ;# pin 5  D0
set_property PACKAGE_PIN F13 [get_ports {jm1[1]}];      set_property PACKAGE_PIN N22 [get_ports {jm2[1]}]   ;# pin 6  D1
set_property PACKAGE_PIN D16 [get_ports {jm1[2]}];      set_property PACKAGE_PIN G18 [get_ports {jm2[2]}]   ;# pin 7  D2
set_property PACKAGE_PIN F14 [get_ports {jm1[3]}];      set_property PACKAGE_PIN M22 [get_ports {jm2[3]}]   ;# pin 8  D3
set_property PACKAGE_PIN C13 [get_ports {jm1[4]}];      set_property PACKAGE_PIN G15 [get_ports {jm2[4]}]   ;# pin 9  D4
set_property PACKAGE_PIN D14 [get_ports {jm1[5]}];      set_property PACKAGE_PIN H17 [get_ports {jm2[5]}]   ;# pin 10 D5
set_property PACKAGE_PIN B13 [get_ports {jm1[6]}];      set_property PACKAGE_PIN G16 [get_ports {jm2[6]}]   ;# pin 11 D6
set_property PACKAGE_PIN D15 [get_ports {jm1[7]}];      set_property PACKAGE_PIN H18 [get_ports {jm2[7]}]   ;# pin 12 D7
set_property PACKAGE_PIN A13 [get_ports {jm1[8]}];      set_property PACKAGE_PIN J14 [get_ports {jm2[8]}]   ;# pin 13 D8
set_property PACKAGE_PIN C14 [get_ports {jm1[9]}];      set_property PACKAGE_PIN J15 [get_ports {jm2[9]}]   ;# pin 14 D9
set_property PACKAGE_PIN A14 [get_ports {jm1[10]}];     set_property PACKAGE_PIN H14 [get_ports {jm2[10]}]  ;# pin 15 D10
set_property PACKAGE_PIN C15 [get_ports {jm1[11]}];     set_property PACKAGE_PIN H15 [get_ports {jm2[11]}]  ;# pin 16 D11
set_property PACKAGE_PIN A15 [get_ports {jm1[12]}];     set_property PACKAGE_PIN H13 [get_ports {jm2[12]}]  ;# pin 17 D12
set_property PACKAGE_PIN E13 [get_ports {jm1[13]}];     set_property PACKAGE_PIN M21 [get_ports {jm2[13]}]  ;# pin 18 D13
set_property PACKAGE_PIN A16 [get_ports {jm1[14]}];     set_property PACKAGE_PIN G13 [get_ports {jm2[14]}]  ;# pin 19 D14
set_property PACKAGE_PIN E14 [get_ports {jm1[15]}];     set_property PACKAGE_PIN L21 [get_ports {jm2[15]}]  ;# pin 20 D15
set_property PACKAGE_PIN B17 [get_ports {jm1[16]}];     set_property PACKAGE_PIN J20 [get_ports {jm2[16]}]  ;# pin 21 VALID
set_property PACKAGE_PIN B15 [get_ports {jm1[17]}];     set_property PACKAGE_PIN H20 [get_ports {jm2[17]}]  ;# pin 22 LAST
set_property PACKAGE_PIN B18 [get_ports {jm1[18]}];     set_property PACKAGE_PIN J21 [get_ports {jm2[18]}]  ;# pin 23 XOFF
set_property PACKAGE_PIN B16 [get_ports {jm1[19]}];     set_property PACKAGE_PIN G20 [get_ports {jm2[19]}]  ;# pin 24 UP
set_property PACKAGE_PIN D17 [get_ports {jm1[20]}];     set_property PACKAGE_PIN J19 [get_ports {jm2[20]}]  ;# pin 25 LCLK (L12P_MRCC)
set_property PACKAGE_PIN F16 [get_ports {jm1[21]}];     set_property PACKAGE_PIN K21 [get_ports {jm2[21]}]  ;# pin 26 PAR
set_property PACKAGE_PIN C17 [get_ports {jm1[22]}];     set_property PACKAGE_PIN H19 [get_ports {jm2[22]}]  ;# pin 27 AUX0
set_property PACKAGE_PIN E17 [get_ports {jm1[23]}];     set_property PACKAGE_PIN K22 [get_ports {jm2[23]}]  ;# pin 28 AUX1
set_property PACKAGE_PIN C18 [get_ports {jm1[24]}];     set_property PACKAGE_PIN K18 [get_ports {jm2[24]}]  ;# pin 29 AUX2
set_property PACKAGE_PIN F18 [get_ports {jm1[25]}];     set_property PACKAGE_PIN J22 [get_ports {jm2[25]}]  ;# pin 30 AUX3
set_property PACKAGE_PIN C19 [get_ports {jm1[26]}];     set_property PACKAGE_PIN K19 [get_ports {jm2[26]}]  ;# pin 31 AUX4
set_property PACKAGE_PIN E18 [get_ports {jm1[27]}];     set_property PACKAGE_PIN H22 [get_ports {jm2[27]}]  ;# pin 32 AUX5
set_property PACKAGE_PIN E19 [get_ports {jm1[28]}];     set_property PACKAGE_PIN L19 [get_ports {jm2[28]}]  ;# pin 37 AUX6
set_property PACKAGE_PIN B20 [get_ports {jm1[29]}];     set_property PACKAGE_PIN M18 [get_ports {jm2[29]}]  ;# pin 38 AUX7
set_property PACKAGE_PIN D19 [get_ports {jm1[30]}];     set_property PACKAGE_PIN L20 [get_ports {jm2[30]}]  ;# pin 39 AUX8
set_property PACKAGE_PIN A20 [get_ports {jm1[31]}];     set_property PACKAGE_PIN L18 [get_ports {jm2[31]}]  ;# pin 40 AUX9

set_property IOSTANDARD LVCMOS33 [get_ports {jm1[*] jm2[*]}]
set_property PULLTYPE PULLDOWN   [get_ports {jm1[*] jm2[*]}]
set_property DRIVE 8             [get_ports {jm1[*] jm2[*]}]
# SLOW on data/control, FAST on the two forwarded clocks (see link_pins.xdc)
set_property SLEW SLOW           [get_ports {jm1[*] jm2[*]}]
set_property SLEW FAST           [get_ports {jm1[20] jm2[20]}]

# TX lane output enable: a one-time static enable (set once after the 100 ms guard window)
set_false_path -from [get_cells -hierarchical -regexp {.*u_link/oe_s_reg\[1\]$}]
# CDC inside link_core: first stage of its 2-FF synchronisers and the async-FIFO gray-pointer syncs
set_false_path -to [get_cells -hierarchical -regexp {.*u_link/.*_s_reg\[[0-9]+\]$}]
set_false_path -to [get_cells -hierarchical -regexp {.*u_link/.*ptr_gray_[wr]1_reg\[[0-9]+\]$}]
