#!/bin/bash
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
# Four-port bench (tb_sdrc_mp.sv) on the board model: builds it with Verilator
# and runs it. The output goes to out/mp_<tag>.txt.
#   usage: build_sdrc_mp.sh <CLK_KHZ> <CORNER> <MAP> <N_CHIPS> <COL_BITS> <CMD_STAGE> <WB2> <LB> <DIR>
#     WB2 = 1: tRRD, tWR, tMRD of two clocks (W9825G6KH) in the controller and the chips
#     LB: reads up to 2^LB-1 halfwords; DIR = 1: the directed tests of the ordering
# VERILATOR, MAKE and PYTHON3 name the tools (defaults: verilator, make, python3).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RTL="$HERE/../rtl"
KHZ="$1"; COR="$2"; MAP="$3"; NCH="$4"; COLB="$5"; CMDS="$6"; WB2="$7"; LB="$8"; DIR="$9"
VL="${VERILATOR:-verilator}"; MK="${MAKE:-make}"; PY="${PYTHON3:-python3}"
TAG="k${KHZ}_c${COR}_m${MAP}_n${NCH}_b${COLB}_s${CMDS}_w${WB2}_lb${LB}_d${DIR}"
OUT="$HERE/obj/mp_$TAG"
mkdir -p "$HERE/tmp/mp_$TAG" "$HERE/out" "$HERE/obj"   # Verilator makes only the last folder of --Mdir
export TMP="$HERE/tmp/mp_$TAG" TEMP="$HERE/tmp/mp_$TAG" TMPDIR="$HERE/tmp/mp_$TAG"
rm -rf "$OUT"
"$VL" --timing -Wno-fatal -Wno-UNOPTFLAT -Wno-SYNCASYNCNET -Wno-WIDTH -Wno-PINMISSING -Wno-lint -Wno-style \
	-cc --exe -O3 --Mdir "$OUT" -CFLAGS "-std=c++20 -O2" \
	-GCLK_KHZ=$KHZ -GCORNER=$COR -GMAP=$MAP -GN_CHIPS=$NCH -GCOL_BITS=$COLB -GCMD_STAGE=$CMDS -GWB2=$WB2 -GLB=$LB -GDIR=$DIR \
	--top-module tb_sdrc_mp "$HERE/tb_sdrc_mp.sv" "$HERE/sdr_chip.sv" "$HERE/sdr_board.sv" "$RTL/sdrc.sv" \
	"$HERE/sim_main_mp.cpp" > "$HERE/out/verilate_mp_$TAG.log" 2>&1 \
	|| { echo "mp_$TAG VERILATE_FAIL"; grep -E "%Error" "$HERE/out/verilate_mp_$TAG.log" | head -20; exit 1; }
"$MK" -C "$OUT" -f Vtb_sdrc_mp.mk PYTHON3="$PY" -j4 > "$HERE/out/make_mp_$TAG.log" 2>&1 \
	|| { echo "mp_$TAG MAKE_FAIL"; grep -E "error|undefined" "$HERE/out/make_mp_$TAG.log" | head -20; exit 2; }
"$OUT/Vtb_sdrc_mp" > "$HERE/out/mp_$TAG.txt" 2>&1
echo "mp_$TAG done"
