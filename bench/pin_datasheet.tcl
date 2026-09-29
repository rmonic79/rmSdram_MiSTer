# This file is part of rmSdram, an SDRAM controller for the MiSTer SDRAM boards.
# Copyright (c) 2026 Umberto Parisi (rmonic79)
#
# rmSdram is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
# later version. It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details: LICENSE in the root of the repository, or <https://www.gnu.org/licenses/>.
#
# A commercial license, for use without the obligations of the GPL (for example in closed-source products), is
# available from the author, who can also be contacted for other licensing needs.
#
# Read-only: clock-to-output and input setup/hold of the SDRAM pins of a
# compiled netlist (no constraints on those pins, so the numbers are the raw
# delays of the I/O paths). From the folder of the project:
#   quartus_sta -t <path to rmSdram>/bench/pin_datasheet.tcl <Project>
set prj [lindex $quartus(args) 0]
project_open $prj
create_timing_netlist
read_sdc
update_timing_netlist
foreach corner {7_slow_1100mv_85c MIN_fast_1100mv_0c} {
	set_operating_conditions $corner
	update_timing_netlist
	puts "=== corner $corner"
	report_datasheet -stdout -panel_name "ds_$corner"
}
delete_timing_netlist
project_close
