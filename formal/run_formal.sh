#!/usr/bin/env bash
# run_formal.sh: run every SymbiYosys proof in this folder and print a summary.
#
#   ./run_formal.sh              all .sby files, all tasks
#   ./run_formal.sh pe_fifo      one file
#
# Each task leaves its work directory at formal/<file>_<task>/ (logs, and a
# counterexample trace in engine_*/ if a property fails).
cd "$(dirname "$0")"
files=("$@")
[ ${#files[@]} -eq 0 ] && files=(*.sby) || files=("${files[@]/%/.sby}")
fail=0
for f in "${files[@]}"; do
  sby -f "$f" >/dev/null 2>&1
done
printf "%-28s %-6s %s\n" "proof" "result" "time"
for f in "${files[@]}"; do
  b=${f%.sby}
  for d in "${b}"_*/; do
    [ -f "$d/status" ] || continue
    st=$(awk '{print $1}' "$d/status")
    t=$(grep -oE "Elapsed clock time \[H:MM:SS \(secs\)\]: [0-9:]+ \([0-9]+\)" "$d/logfile.txt" | tail -1 | grep -oE "\([0-9]+\)" | tr -d '()')
    printf "%-28s %-6s %ss\n" "${d%/}" "$st" "$t"
    [ "$st" = PASS ] || fail=1
  done
done
exit $fail
