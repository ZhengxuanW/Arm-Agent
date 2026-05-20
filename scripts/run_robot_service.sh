#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR/mycobot_sorting_package"

if [[ ! -x "$ROOT_DIR/.venv/bin/python" ]]; then
  echo "[run] .venv missing. Run scripts/setup_env.sh first." >&2
  exit 1
fi

export MYCobot_PORT="${MYCobot_PORT:-/dev/cu.usbserial-1130}"
export ROBOT_SERVICE_HOST="${ROBOT_SERVICE_HOST:-127.0.0.1}"
export ROBOT_SERVICE_PORT="${ROBOT_SERVICE_PORT:-5050}"

echo "[run] port: $MYCobot_PORT"
echo "[run] service: http://$ROBOT_SERVICE_HOST:$ROBOT_SERVICE_PORT"

exec "$ROOT_DIR/.venv/bin/python" robot_service.py --host "$ROBOT_SERVICE_HOST" --port "$ROBOT_SERVICE_PORT"
