# SPDX-License-Identifier: Apache-2.0
"""
coverage_report: merge Verilator coverage and print a per-file summary.

Usage (in the tt-dev-cov image):
    python3 coverage_report.py OUT_DIR file1.dat [file2.dat ...]
Writes OUT_DIR/merged.dat, OUT_DIR/merged.info (lcov) and OUT_DIR/annotated/
(sources with per-line hit counts; uncovered lines start with '%').
Prints line / branch / toggle coverage per RTL file as the union over all
instances and tests, the uncovered line and branch points, and the signals
with bits that never toggled.
"""

import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

out = Path(sys.argv[1])
dats = sys.argv[2:]
out.mkdir(parents=True, exist_ok=True)
merged = out / "merged.dat"
subprocess.run(["verilator_coverage", "--write", str(merged)] + dats, check=True)
subprocess.run(["verilator_coverage", "--write-info", str(out / "merged.info"), str(merged)], check=True)
subprocess.run(["verilator_coverage", "--annotate", str(out / "annotated"), "--annotate-min", "1",
                str(merged)], check=True, capture_output=True)



def points(path):
    for ln in Path(path).read_text().splitlines():
        if not ln.startswith("C '"):
            continue
        body, cnt = ln[3:].rsplit("' ", 1)
        kv = dict(p.split("\x02", 1) for p in body.split("\x01") if "\x02" in p)
        yield kv, int(cnt)


# Union over instances: a point (file, line, type, name) is covered if any
# instance in any test hit it (module tests and the chip share the RTL).
pts = defaultdict(int)
for kv, cnt in points(merged):
    f = kv.get("f", "")
    if "/src/" not in f:
        continue
    t = kv.get("t", kv.get("page", "?").split("/")[0].replace("v_", ""))
    key = (f.split("/src/")[1], int(kv.get("l", 0)), t, kv.get("o", ""))
    pts[key] += cnt

per = defaultdict(lambda: defaultdict(lambda: [0, 0]))
untoggled = defaultdict(set)
miss_lines = defaultdict(set)
for (name, line, t, o), cnt in pts.items():
    per[name][t][1] += 1
    if cnt > 0:
        per[name][t][0] += 1
    elif t == "toggle":
        untoggled[name].add(re.sub(r"\[\d+\]", "[]", o.split(":")[0]))
    else:
        miss_lines[name].add(line)

types = sorted({t for d in per.values() for t in d})
print(f"{'file':28s}" + "".join(f"{t:>18s}" for t in types))
tot = defaultdict(lambda: [0, 0])
for name in sorted(per):
    row = f"{name:28s}"
    for t in types:
        h, n = per[name][t]
        tot[t][0] += h
        tot[t][1] += n
        row += f"{(f'{h}/{n} ' + (f'{100 * h / n:5.1f}%' if n else '  -  ')):>18s}"
    print(row)
print(f"{'TOTAL':28s}" + "".join(f"{f'{tot[t][0]}/{tot[t][1]} {100 * tot[t][0] / max(1, tot[t][1]):5.1f}%':>18s}"
                                for t in types))
if "--detail" in sys.argv[0:1] or True:
    print("\nuncovered line/branch points:")
    for name in sorted(miss_lines):
        src = (Path(__file__).resolve().parents[1] / "src" / name).read_text().splitlines()
        for n in sorted(miss_lines[name]):
            print(f"  {name}:{n}: {src[n - 1].strip()[:90] if n else ''}")
    print("\nsignals with a bit that never toggled (scratch / constant-by-design, or a gap):")
    for name in sorted(untoggled):
        print(f"--- {name}: " + ", ".join(sorted(untoggled[name])))
