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
# Checks the outputs of regress_sdrc.sh (out/): every run ends clean, and the
# result lines are the same as the reference ones (ref/), in the same order.
#   usage: python3 compare_sdrc.py            compare
#          python3 compare_sdrc.py --update   write ref/ from out/ (after a change meant to alter the figures)
import os, re, sys
H = os.path.dirname(os.path.abspath(__file__))
checks = [
    ('tb_sdrc (1 port)',     'tb_sdrc.txt',    r'^(== |READY|W_BTB|W_GAP|R_LINE|R_ONE|MIXED|REFRESH|\[U2\] no chip|CHECK|MISSING)'),
    ('tb_sdrc_mp (4 ports)', 'tb_sdrc_mp.txt', r'^(== |READY|P[0-3] |SLOT|BUS|REFRESH|DIRECTED|NCS|CHECK|MISSING)'),
    ('tb_sdrc_job (jobs)',   'tb_sdrc_job.txt', r'^(== |READY|JOBS|WORDS|\[U[12]\]|CHECK|MISSING)'),
]
def pick(path, rx):
    t = open(path, encoding='utf-8', errors='replace').read()
    return [l.rstrip() for l in t.splitlines() if re.match(rx, l)]

if '--update' in sys.argv:
    # the reference keeps the result lines only (no paths or tool messages)
    os.makedirs(os.path.join(H, 'ref'), exist_ok=True)
    for _, fn, rx in checks:
        with open(os.path.join(H, 'ref', fn), 'w', encoding='utf-8', newline='\n') as f:
            f.write('\n'.join(pick(os.path.join(H, 'out', fn), rx)) + '\n')
        print('ref/' + fn + ' written')
    sys.exit(0)

ok = True
for name, fn, rx in checks:
    ln = pick(os.path.join(H, 'out', fn), rx)
    runs  = [l for l in ln if l.startswith('== ')]
    clean = [l for l in ln if l.startswith('CHECK') and
             re.search(r'data_errors=0\b', l) and re.search(r'chip_errors=0\b', l) and
             re.search(r'bench_errors=0\b', l) and not re.search(r'refresh_exact=0', l) and
             not re.search(r'protocol_errors=[1-9]', l) and
             not re.search(r'lost=[1-9]|underflows=[1-9]|audio_late=[1-9]', l)]
    miss  = [l for l in ln if l.startswith('MISSING')]
    run_ok = len(runs) > 0 and len(clean) == len(runs) and not miss
    lo = pick(os.path.join(H, 'ref', fn), rx) if os.path.exists(os.path.join(H, 'ref', fn)) else []
    same = (lo == ln) and len(lo) > 0
    ok = ok and run_ok and same
    print('%-22s runs %d, clean %d, reference %s (%d lines)' %
          (name, len(runs), len(clean), 'IDENTICAL' if same else 'DIFFERENT', len(lo)))
    if not same:
        shown = 0
        for a, b in zip(lo, ln):
            if a != b and shown < 8:
                print('   ref: ' + a); print('   new: ' + b); shown += 1
        if len(lo) != len(ln): print('   ref has %d lines, new %d' % (len(lo), len(ln)))
print('ALL CLEAN AND IDENTICAL' if ok else 'NOT ALL CLEAN AND IDENTICAL')
sys.exit(0 if ok else 1)
