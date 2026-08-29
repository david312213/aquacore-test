#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="aquacore-test"

if ! command -v conda >/dev/null 2>&1; then
  echo "Conda was not found." >&2
  echo "Install Miniconda, reopen the terminal, then run this script again." >&2
  echo "Official installer: https://www.anaconda.com/docs/getting-started/miniconda/install/" >&2
  exit 1
fi

if conda run -n "$ENV_NAME" python --version >/dev/null 2>&1; then
  conda env update --name "$ENV_NAME" --file environment.yml --prune
else
  conda env create --file environment.yml
fi

conda run -n "$ENV_NAME" python -m unittest discover -s tests -v

echo
echo "Setup complete. Try:"
echo "  conda run -n $ENV_NAME python run_sim.py --controller student --seed 1001 --animate --events"
