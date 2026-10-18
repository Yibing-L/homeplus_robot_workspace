"""Self-test for server.py: /health, then /point and /segment N times on one image.

Usage: python test_client.py IMAGE [caption] [url]
Writes test_overlay.jpg (point + mask) next to this script.
"""
import base64
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import requests

image_path = sys.argv[1]
caption = sys.argv[2] if len(sys.argv) > 2 else 'red cup'
url = sys.argv[3] if len(sys.argv) > 3 else 'http://127.0.0.1:8765'

image = cv2.resize(cv2.imread(image_path), (640, 480))  # robot frame size
ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
b64 = base64.b64encode(jpg.tobytes()).decode('ascii')

print('health:', json.dumps(requests.get(f'{url}/health').json()))

for i in range(3):
    t0 = time.perf_counter()
    p = requests.post(f'{url}/point', json={'image_jpeg_b64': b64, 'caption': caption}).json()
    rt_point = time.perf_counter() - t0
    print(f'[{i}] /point server={p["elapsed_sec"]:.3f}s roundtrip={rt_point:.3f}s '
          f'points={p["points"]} text={p["generated_text"]!r}')
    if not p['points']:
        continue
    u, v = p['points'][0][2], p['points'][0][3]
    t0 = time.perf_counter()
    s = requests.post(f'{url}/segment', json={'image_jpeg_b64': b64, 'point': [u, v]}).json()
    rt_seg = time.perf_counter() - t0
    print(f'[{i}] /segment server={s["elapsed_sec"]:.3f}s roundtrip={rt_seg:.3f}s score={s["score"]}')

if p['points'] and s['mask_png_b64']:
    mask = cv2.imdecode(np.frombuffer(base64.b64decode(s['mask_png_b64']), np.uint8),
                        cv2.IMREAD_UNCHANGED)
    ys, xs = np.nonzero(mask)
    print(f'mask shape={mask.shape} dtype={mask.dtype} area={len(xs)} '
          f'bbox=({xs.min()},{ys.min()})-({xs.max()},{ys.max()})')
    overlay = image.copy()
    overlay[mask > 0] = (0.5 * overlay[mask > 0] + [0, 127, 0]).astype(np.uint8)
    for _, _, x, y in p['points']:
        cv2.circle(overlay, (int(x), int(y)), 6, (255, 0, 0), -1)
    cv2.imwrite(str(Path(__file__).with_name('test_overlay.jpg')), overlay)
