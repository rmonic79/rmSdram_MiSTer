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
# Every bench of the controller, every configuration: builds and runs them
# (JOBS at a time, default 4), collects the outputs in configuration order into
# out/tb_sdrc.txt, out/tb_sdrc_mp.txt and out/tb_sdrc_job.txt, and compares their result lines with
# ref/ (compare_sdrc.py): the same lines, in the same order, must come out, and
# every run must end clean.
#   usage: regress_sdrc.sh
# VERILATOR, MAKE and PYTHON3 name the tools (defaults: verilator, make, python3).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE" || exit 9
mkdir -p out
# one port: CLK_KHZ CORNER N_CHIPS COL_BITS WB2
cat > out/cfg_sdrc.txt <<'EOF'
48000 0 2 10 0
48000 1 2 10 0
66667 0 2 10 0
66667 1 2 10 0
48000 0 1 9 1
48000 1 1 9 1
48000 0 1 10 0
EOF
# four ports: CLK_KHZ CORNER MAP N_CHIPS COL_BITS CMD_STAGE WB2 LB DIR
#   two chips, both maps, both clocks, both corners; reads up to 2047 halfwords;
#   one chip with 9 column bits (W9825G6KH timings) and with 10
cat > out/cfg_mp.txt <<'EOF'
48000 0 1 2 10 1 0 6 0
48000 1 1 2 10 1 0 6 0
66667 0 1 2 10 1 0 6 0
66667 1 1 2 10 1 0 6 0
48000 0 0 2 10 1 0 6 0
48000 1 0 2 10 1 0 6 0
66667 0 0 2 10 1 0 6 0
66667 1 0 2 10 1 0 6 0
48000 0 1 2 10 1 0 11 1
66667 0 0 2 10 0 0 11 1
48000 0 1 1 9 0 1 10 1
48000 1 1 1 9 0 1 10 1
48000 0 0 1 9 0 1 10 1
66667 0 1 1 9 1 1 10 1
66667 0 0 1 9 1 1 10 1
48000 0 1 1 10 0 0 11 1
48000 0 0 1 10 0 0 6 1
EOF
# job port: CLK_KHZ CORNER N_CHIPS COL_BITS WB2 BL_BITS LEN_BITS FIXED1
#   jobs longer than the sdrc port (split reads), jobs that fit it, one word (FIXED1)
cat > out/cfg_job.txt <<'EOF'
48000 0 2 10 0 9 4 0
48000 1 2 10 0 9 4 0
66667 0 2 10 0 9 4 0
48000 0 1 9 1 9 4 0
48000 0 1 9 1 3 4 0
48000 0 2 10 0 4 4 1
EOF
xargs -P "${JOBS:-4}" -L 1 bash "$HERE/build_sdrc.sh" < out/cfg_sdrc.txt
xargs -P "${JOBS:-4}" -L 1 bash "$HERE/build_sdrc_mp.sh" < out/cfg_mp.txt
xargs -P "${JOBS:-4}" -L 1 bash "$HERE/build_sdrc_job.sh" < out/cfg_job.txt
while read k c n b w; do
	t="k${k}_c${c}_n${n}_b${b}_w${w}"
	echo "== $k kHz corner $c chips $n column bits $b two-clock $w"
	cat "out/sdrc_$t.txt" 2>/dev/null || echo "MISSING out/sdrc_$t.txt"
done < out/cfg_sdrc.txt > out/tb_sdrc.txt
while read k c m n b s w lb d; do
	t="k${k}_c${c}_m${m}_n${n}_b${b}_s${s}_w${w}_lb${lb}_d${d}"
	echo "== $k kHz corner $c MAP $m chips $n column bits $b command stage $s two-clock $w LB $lb directed $d"
	cat "out/mp_$t.txt" 2>/dev/null || echo "MISSING out/mp_$t.txt"
done < out/cfg_mp.txt > out/tb_sdrc_mp.txt
while read k c n b w bl ln f; do
	t="k${k}_c${c}_n${n}_b${b}_w${w}_bl${bl}_ln${ln}_f${f}"
	echo "== $k kHz corner $c chips $n column bits $b two-clock $w BL_BITS $bl LEN_BITS $ln FIXED1 $f"
	cat "out/job_$t.txt" 2>/dev/null || echo "MISSING out/job_$t.txt"
done < out/cfg_job.txt > out/tb_sdrc_job.txt
"${PYTHON3:-python3}" "$HERE/compare_sdrc.py"
