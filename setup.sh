#!/usr/bin/env bash
# Create the virtualenv the module is built and tested with (Viam's cloud build runs this first).
set -euo pipefail
cd "$(dirname "$0")"

if ! python3 -m venv --help > /dev/null 2>&1; then
  echo "python3 -m venv isn't available; install python3-venv (e.g. sudo apt-get install python3-venv)" >&2
  exit 1
fi
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements-dev.txt
