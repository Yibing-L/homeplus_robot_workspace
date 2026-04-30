#!/usr/bin/env python3
"""
One-shot: load a torch checkpoint that was pickled under numpy 2.x and
re-save it with no numpy refs in the pickle stream, so it loads cleanly
under numpy 1.x without needing PYTHONPATH overrides.

Run with PYTHONPATH set to a numpy>=2.0 install:
    PYTHONPATH=$HOME/.local/homeplus_gesture_pkgs python3 \\
        scripts/convert_checkpoint_np_compat.py \\
        /abs/in.pt /abs/out.pt
"""

import sys
from pathlib import Path

import numpy as np
import torch


def to_np1_compat(obj):
    """Recursively replace numpy arrays/scalars with torch tensors / Python."""
    if isinstance(obj, np.ndarray):
        return torch.from_numpy(np.array(obj))
    if isinstance(obj, (np.generic,)):
        return obj.item()
    if isinstance(obj, dict):
        return {k: to_np1_compat(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [to_np1_compat(v) for v in obj]
    if isinstance(obj, tuple):
        return tuple(to_np1_compat(v) for v in obj)
    return obj


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    if not src.is_file():
        sys.exit(f"input not found: {src}")

    print(f"numpy: {np.__version__}, torch: {torch.__version__}")
    print(f"loading {src} ...")
    artifact = torch.load(src, map_location="cpu", weights_only=False)
    print(f"top-level keys: {sorted(artifact.keys())}")

    cleaned = to_np1_compat(artifact)
    torch.save(cleaned, dst)
    print(f"wrote {dst} ({dst.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
