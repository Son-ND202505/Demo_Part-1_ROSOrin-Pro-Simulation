#!/usr/bin/env bash
# Build the unchanged vendor packages in this repository's dedicated workspace.
set -eo pipefail
PART1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PART1_ROOT"
source /opt/ros/humble/setup.bash
mkdir -p part1_rosorin/ws part1_rosorin/results/audit
colcon --log-base part1_rosorin/ws/log build \
  --base-paths third_party/rosorin_pro/upstream/src/simulations \
  --build-base part1_rosorin/ws/build \
  --install-base part1_rosorin/ws/install \
  --event-handlers console_direct+ \
  2>&1 | tee part1_rosorin/results/audit/build.log
