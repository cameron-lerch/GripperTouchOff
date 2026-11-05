#!/usr/bin/env bash
# Build the module into one executable (PyInstaller bundles Python and the dependencies, so the
# machine needs no Python) and package it with meta.json.
# PyInstaller doesn't cross-compile: this builds for the machine it runs on; Viam's cloud build
# (`viam module build start`) runs it on each platform in meta.json.
set -euo pipefail
cd "$(dirname "$0")"

[ -x .venv/bin/python ] || ./setup.sh
rm -rf build dist
.venv/bin/python -m PyInstaller --onefile --name main --distpath dist --workpath build --specpath build \
  --paths src --collect-submodules touchoff src/main.py
tar -czf dist/archive.tar.gz meta.json dist/main
echo "Built dist/archive.tar.gz"
