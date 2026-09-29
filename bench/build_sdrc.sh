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
# One-port bench (tb_sdrc.sv) on the board model: builds it with Verilator and
# runs it. The output goes to out/sdrc_<tag>.txt.
#   usage: build_sdrc.sh <CLK_KHZ> <CORNER 0=slow 1=fast> [N_CHIPS] [COL_BITS] [WB2]
# VERILATOR, MAKE and PYTHON3 name the tools (defaults: verilator, make, python3).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RTL="$HERE/../rtl"
KHZ="$1"; COR="$2"; NCH="${3:-2}"; COLB="${4:-10}"; WB2="${5:-0}"
VL="${VERILATOR:-verilator}"; MK="${MAKE:-make}"; PY="${PYTHON3:-python3}"
TAG="k${KHZ}_c${COR}_n${NCH}_b${COLB}_w${WB2}"
OUT="$HERE/obj/sdrc_$TAG"
mkdir -p "$HERE/tmp/sdrc_$TAG" "$HERE/out" "$HERE/obj"   # Verilator makes only the last folder of --Mdir
# the compiler needs a writable temporary directory whatever the environment
export TMP="$HERE/tmp/sdrc_$TAG" TEMP="$HERE/tmp/sdrc_$TAG" TMPDIR="$HERE/tmp/sdrc_$TAG"
rm -rf "$OUT"
"$VL" --timing -Wno-fatal -Wno-UNOPTFLAT -Wno-SYNCASYNCNET -Wno-WIDTH -Wno-PINMISSING -Wno-lint -Wno-style \
	-cc --exe -O3 --Mdir "$OUT" -CFLAGS "-std=c++20 -O2" \
	-GCLK_KHZ=$KHZ -GCORNER=$COR -GN_CHIPS=$NCH -GCOL_BITS=$COLB -GWB2=$WB2 \
	--top-module tb_sdrc "$HERE/tb_sdrc.sv" "$HERE/sdr_chip.sv" "$HERE/sdr_board.sv" "$RTL/sdrc.sv" \
	"$HERE/sim_main_sdrc.cpp" > "$HERE/out/verilate_sdrc_$TAG.log" 2>&1 \
	|| { echo "sdrc_$TAG VERILATE_FAIL"; grep -E "%Error" "$HERE/out/verilate_sdrc_$TAG.log" | head -20; exit 1; }
"$MK" -C "$OUT" -f Vtb_sdrc.mk PYTHON3="$PY" -j4 > "$HERE/out/make_sdrc_$TAG.log" 2>&1 \
	|| { echo "sdrc_$TAG MAKE_FAIL"; grep -E "error|undefined" "$HERE/out/make_sdrc_$TAG.log" | head -20; exit 2; }
"$OUT/Vtb_sdrc" > "$HERE/out/sdrc_$TAG.txt" 2>&1
echo "sdrc_$TAG done"
