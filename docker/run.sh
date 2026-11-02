#!/usr/bin/env bash
# Auto-detects cpu/cuda/l4t and brings up the matching compose file.
# Mirrors vendor/home2/run.sh's pattern. Usage:
#   ./docker/run.sh            # auto-detect
#   ./docker/run.sh cpu|cuda|l4t   # force a variant
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_DIR="${SCRIPT_DIR}/compose"

detect_variant() {
  if [ -f /etc/nv_tegra_release ]; then
    echo "l4t"
  elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    echo "cuda"
  else
    echo "cpu"
  fi
}

VARIANT="${1:-$(detect_variant)}"

case "$VARIANT" in
  cpu|cuda|l4t) ;;
  *)
    echo "Unknown variant: $VARIANT (use cpu, cuda, or l4t)" >&2
    exit 1
    ;;
esac

echo "Bringing up HomeHumanoid with variant: ${VARIANT}"
docker compose -f "${COMPOSE_DIR}/${VARIANT}.yaml" up --build -d
docker compose -f "${COMPOSE_DIR}/${VARIANT}.yaml" exec homehumanoid bash
