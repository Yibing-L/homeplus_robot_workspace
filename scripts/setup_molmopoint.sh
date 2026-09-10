#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${MOLMOPOINT_VENV:-$WORKSPACE_ROOT/.venv-molmopoint}"
MODEL_CACHE="${MOLMOPOINT_CACHE:-$WORKSPACE_ROOT/.models/huggingface}"
MODEL_ID="${MOLMOPOINT_MODEL:-allenai/MolmoPoint-8B}"
MODEL_REVISION="${MOLMOPOINT_REVISION:-188130f961c8e0888a34e11121a1423c461a01ba}"

if [[ ! -x "$VENV_DIR/bin/python" || ! -x "$VENV_DIR/bin/pip" ]]; then
  if ! python3 -m venv --system-site-packages "$VENV_DIR"; then
    echo "python3-venv is unavailable; bootstrapping virtualenv without sudo." >&2
    BOOTSTRAP_DIR="$(mktemp -d /tmp/homeplus-virtualenv.XXXXXX)"
    python3 -m pip install --target "$BOOTSTRAP_DIR" virtualenv
    PYTHONPATH="$BOOTSTRAP_DIR" python3 -m virtualenv \
      --system-site-packages "$VENV_DIR"
  fi
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r "$WORKSPACE_ROOT/requirements-molmopoint.txt"

"$VENV_DIR/bin/python" - <<'PY'
import torch
import transformers
import accelerate
import bitsandbytes
import decord
import jinja2

if transformers.__version__ != '4.57.1':
    raise SystemExit(f'Expected transformers 4.57.1, got {transformers.__version__}')
if tuple(map(int, jinja2.__version__.split('.')[:2])) < (3, 1):
    raise SystemExit(f'Expected jinja2 >= 3.1, got {jinja2.__version__}')
if not torch.cuda.is_available():
    raise SystemExit(
        'CUDA is not available to PyTorch. Fix the NVIDIA runtime before launching.'
    )
print(
    f'Runtime verified: torch={torch.__version__}, '
    f'transformers={transformers.__version__}, '
    f'accelerate={accelerate.__version__}, '
    f'bitsandbytes={bitsandbytes.__version__}, '
    f'decord2={decord.__version__}, '
    f'jinja2={jinja2.__version__}, '
    f'gpu={torch.cuda.get_device_name(0)}'
)
PY

mkdir -p "$MODEL_CACHE"

if [[ "${1:-}" == "--download-model" ]]; then
  MOLMOPOINT_MODEL_ID="$MODEL_ID" \
    MOLMOPOINT_MODEL_REVISION="$MODEL_REVISION" \
    MOLMOPOINT_MODEL_CACHE="$MODEL_CACHE" \
    "$VENV_DIR/bin/python" - <<'PY'
import os
from huggingface_hub import snapshot_download

model_id = os.environ['MOLMOPOINT_MODEL_ID']
cache_dir = os.environ['MOLMOPOINT_MODEL_CACHE']
revision = os.environ['MOLMOPOINT_MODEL_REVISION']
print(f'Downloading {model_id}@{revision} into {cache_dir} ...')
path = snapshot_download(repo_id=model_id, revision=revision, cache_dir=cache_dir)
print(f'MolmoPoint checkpoint ready: {path}')
PY
fi

echo "MolmoPoint environment ready: $VENV_DIR"
echo "Model cache: $MODEL_CACHE"
if [[ "${1:-}" != "--download-model" ]]; then
  echo "Use --download-model to populate the pinned checkpoint when needed."
fi
