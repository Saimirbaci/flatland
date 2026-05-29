#!/usr/bin/env bash
#
# solver_tuning_sweep.sh - sweep Box2D contact-solver settings against the
# Flatland benchmark and report throughput (iterations/sec) for each combo.
#
# Stability (penetration / tunnelling) is validated separately by the gtest
# rostest `solver_stability_test`; this script measures the *throughput cost* of
# each solver configuration so the two together give the stability-vs-speed
# trade-off used to pick AGV-scale defaults (see
# flatland_server/doc/contact_solver.md).
#
# Requires a built + sourced catkin workspace (source devel/setup.bash).
#
# Usage:
#   scripts/solver_tuning_sweep.sh [benchmark_duration_sec]
#
set -euo pipefail

DURATION="${1:-30.0}"

# Locate the package and a base world to sweep. We copy the standard
# benchmark_world so we sweep against a realistic map, then rewrite only its
# `properties:` block per combo.
PKG_DIR="$(rospack find flatland_server)"
BASE_WORLD="${PKG_DIR}/test/benchmark_world"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT
cp -r "${BASE_WORLD}/." "${TMP_DIR}/"
SWEEP_WORLD="${TMP_DIR}/world.yaml"

# The grid to sweep. Edit these to taste.
SUBSTEPS_LIST=(1 2 4 8)
VEL_ITERS_LIST=(8 10 12)
POS_ITERS_LIST=(3 8)
CONTINUOUS_LIST=(true false)

printf "%-9s %-9s %-9s %-12s %-12s\n" \
  "substeps" "vel_it" "pos_it" "continuous" "iter/sec"
printf -- "----------------------------------------------------------\n"

# Everything in the original world.yaml after the properties block is preserved;
# we regenerate just the properties header each run.
TAIL_FILE="${TMP_DIR}/tail.yaml"
awk 'f{print} /^layers:/{print; f=1}' "${BASE_WORLD}/world.yaml" > "${TAIL_FILE}" || true
# Fallback: if the awk above produced nothing (unexpected layout), keep the
# original body verbatim minus a leading properties line.
if [[ ! -s "${TAIL_FILE}" ]]; then
  grep -v '^properties:' "${BASE_WORLD}/world.yaml" > "${TAIL_FILE}"
fi

for sub in "${SUBSTEPS_LIST[@]}"; do
  for vel in "${VEL_ITERS_LIST[@]}"; do
    for pos in "${POS_ITERS_LIST[@]}"; do
      for cont in "${CONTINUOUS_LIST[@]}"; do
        {
          echo "properties:"
          echo "  velocity_iterations: ${vel}"
          echo "  position_iterations: ${pos}"
          echo "  substeps: ${sub}"
          echo "  continuous_physics: ${cont}"
          echo "layers:"
          cat "${TAIL_FILE}"
        } > "${SWEEP_WORLD}"

        OUT="$(roslaunch flatland_server benchmark.launch \
                 world_path:="${SWEEP_WORLD}" \
                 benchmark_duration:="${DURATION}" \
                 show_viz:=false 2>&1 || true)"

        IPS="$(echo "${OUT}" | grep -oE '[0-9]+\.[0-9]+ iter/sec' \
                 | tail -1 | grep -oE '[0-9]+\.[0-9]+' || echo "n/a")"

        printf "%-9s %-9s %-9s %-12s %-12s\n" \
          "${sub}" "${vel}" "${pos}" "${cont}" "${IPS}"
      done
    done
  done
done
