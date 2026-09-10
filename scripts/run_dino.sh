#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "run_dino.sh is deprecated; starting MolmoPoint." >&2
exec bash "$ROOT/scripts/run_molmopoint.sh" "$@"
