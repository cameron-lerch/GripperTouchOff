#!/usr/bin/env bash
# Run the module from source with the virtualenv made by setup.sh (local development).
cd "$(dirname "$0")"
exec .venv/bin/python src/main.py "$@"
