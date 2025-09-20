#!/usr/bin/env bash
# k sweep for CBS / ECBS / EECBS with summary CSV
# Features:
#  - CBS only for k <= CBS_MAX_K (default 50). Over this, record NA.
#  - Wall-clock timeout (default 600s) via gtimeout/timeout if available.

# Treat unset variables as errors when expanded
set -u

########################################
# Experiment parameters (can override via env)
########################################
# Map / scenario
MAP="${MAP:-random-32-32-20.map}"
SCEN="${SCEN:-random-32-32-20-random-1.scen}"

# Internal solver time limit (seconds)
TL="${TL:-120}"

# Suboptimality for ECBS/EECBS
W="${W:-1.3}"

# Set of k values to try (space-separated)
K_LIST="${K_LIST:-20 50 80 100 150}"

# CBS will be run only up to this k
CBS_MAX_K="${CBS_MAX_K:-50}"

# Wall-clock timeout seconds (external)
WT="${WT:-600}"

# Binary and output dir
BIN="./build/eecbs"
OUTDIR="./runs_k_sweep"
mkdir -p "$OUTDIR"

########################################
# Detect timeout command
########################################
TIMEOUT_BIN="${TIMEOUT_BIN:-}"
if command -v gtimeout >/dev/null 2>&1; then
  TIMEOUT_BIN="gtimeout"
elif command -v timeout >/dev/null 2>&1; then
  TIMEOUT_BIN="timeout"
else
  TIMEOUT_BIN=""
  echo "NOTE: gtimeout/timeout not found. Wall-clock timeout disabled (WT=${WT}s not applied)." >&2
fi

########################################
# Init summary CSV
########################################
SUMMARY="$OUTDIR/summary.csv"
echo "mode,k,SOC,time" > "$SUMMARY"

########################################
# Run one configuration
########################################
run_one () {
  local mode="$1" k="$2" extra_flags="$3" subopt="$4"
  local tag="${mode}_k${k}"
  local stats="${OUTDIR}/stats_${tag}.csv"
  local paths="${OUTDIR}/paths_${tag}.txt"
  local log="${OUTDIR}/log_${tag}.txt"

  echo ">>> [${mode}] k=${k}, w=${subopt}, TL=${TL}s, WT=${WT}s"

  set +e
  if [[ -n "$TIMEOUT_BIN" ]]; then
    "$TIMEOUT_BIN" "$WT" \
      "$BIN" -m "$MAP" -a "$SCEN" -k "$k" -t "$TL" \
      --suboptimality="$subopt" \
      $extra_flags \
      -o "$stats" \
      --outputPaths="$paths" | tee "$log"
  else
    "$BIN" -m "$MAP" -a "$SCEN" -k "$k" -t "$TL" \
      --suboptimality="$subopt" \
      $extra_flags \
      -o "$stats" \
      --outputPaths="$paths" | tee "$log"
  fi
  local rc=$?
  set -e

  # Failure / timeout / no stats file => NA
  if [[ $rc -ne 0 || ! -s "$stats" ]]; then
    echo "WARN: ${mode} k=${k} failed (timeout/no-solution/error)" >&2
    echo "${mode},${k},NA,NA" >> "$SUMMARY"
    return
  fi

  # Extract SOC (col2) and time (col3) from last line
  local last soc time
  last=$(tail -n 1 "$stats")
  soc=$(echo "$last"  | awk -F, '{print $2}')
  time=$(echo "$last" | awk -F, '{print $3}')
  echo "${mode},${k},${soc},${time}" >> "$SUMMARY"
}

########################################
# Print conditions
########################################
echo "=== CONDITIONS ==="
echo "MAP=${MAP}"
echo "SCEN=${SCEN}"
echo "K_LIST=${K_LIST:-}"
echo "CBS_MAX_K=${CBS_MAX_K:-}, TL=${TL:-}, WT=${WT:-}, TIMEOUT_BIN=${TIMEOUT_BIN:-none}"
echo

########################################
# Main loop
########################################
for k in $K_LIST; do
  # 1) CBS (optimal) — skip large k
  if [[ "$k" -le "$CBS_MAX_K" ]]; then
    run_one "CBS" "$k" "" "1.0"
  else
    echo ">>> [CBS] skip k=${k} (CBS_MAX_K=${CBS_MAX_K})"
    echo "CBS,${k},NA,NA" >> "$SUMMARY"
  fi

  # 2) ECBS (approx, reasoning OFF)
  run_one "ECBS" "$k" "--bypass=0 --rectangleReasoning=0 --corridorReasoning=0 --targetReasoning=0" "$W"

  # 3) EECBS (approx, reasoning ON = default)
  run_one "EECBS" "$k" "" "$W"
done

echo
echo "=== DONE: ${SUMMARY} ==="
if command -v column >/dev/null 2>&1; then
  column -s, -t "$SUMMARY" || cat "$SUMMARY"
else
  cat "$SUMMARY"
fi