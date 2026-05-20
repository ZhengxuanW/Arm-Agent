#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[setup] root: $ROOT_DIR"

if command -v uv >/dev/null 2>&1; then
  echo "[setup] using uv: $(command -v uv)"
  CREATE_VENV=1
  if [[ -x .venv/bin/python ]]; then
    VENV_VERSION="$(.venv/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    if [[ "$VENV_VERSION" != "3.12" ]]; then
      echo "[setup] replacing incompatible .venv Python $VENV_VERSION with Python 3.12"
      rm -rf .venv
    else
      CREATE_VENV=0
    fi
  elif [[ -d .venv ]]; then
    echo "[setup] replacing incomplete .venv with Python 3.12"
    rm -rf .venv
  fi
  if [[ "$CREATE_VENV" == "1" ]]; then
    uv venv --python 3.12 .venv
  fi
else
  PYTHON_BIN="${PYTHON_BIN:-}"
  if [[ -z "$PYTHON_BIN" ]]; then
    if command -v python3.12 >/dev/null 2>&1; then
      PYTHON_BIN="python3.12"
    else
      echo "[setup] Python 3.12 is required. Install python3.12 or uv, then rerun this script." >&2
      exit 1
    fi
  fi

  PYTHON_VERSION="$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  if [[ "$PYTHON_VERSION" != "3.12" ]]; then
    echo "[setup] Python 3.12 is required, got $PYTHON_VERSION from $PYTHON_BIN" >&2
    exit 1
  fi

  echo "[setup] python: $($PYTHON_BIN --version)"
  if [[ ! -d .venv ]]; then
    "$PYTHON_BIN" -m venv .venv
  fi
fi

VENV_PY="$ROOT_DIR/.venv/bin/python"
"$VENV_PY" -m ensurepip --upgrade >/dev/null 2>&1 || true
"$VENV_PY" -m pip install --upgrade pip setuptools wheel
"$VENV_PY" -m pip install -e .

"$VENV_PY" - <<'PY'
import importlib

required = [
    "flask",
    "cv2",
    "numpy",
    "pymycobot",
]

missing = []
for name in required:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append((name, repr(exc)))

if missing:
    for name, exc in missing:
        print(f"[setup] missing {name}: {exc}")
    raise SystemExit(1)

print("[setup] dependency import check ok")
PY

echo "[setup] done"
echo "[setup] run service with: scripts/run_robot_service.sh"
