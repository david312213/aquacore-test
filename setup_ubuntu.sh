#!/usr/bin/env bash
set -euo pipefail

if ! command -v python3 >/dev/null 2>&1; then
  echo "Python 3 is required. Install it with:" >&2
  echo "  sudo apt update && sudo apt install python3 python3-venv python3-pip" >&2
  exit 1
fi

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v

echo
echo "Setup complete. Try:"
echo "  .venv/bin/python run_sim.py --controller student --seed 1001 --animate"
