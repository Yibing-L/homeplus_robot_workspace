#!/usr/bin/env python3
"""Run MolmoPoint-8B once on an image using Apple Silicon (MPS), mirroring
MolmoPointNode.run_model_inference, and report the points and timing.

Usage: python molmopoint_mac_smoke.py IMAGE [CAPTION] [--device mps|cpu]
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor
from huggingface_hub import snapshot_download

MODEL_ID = 'allenai/MolmoPoint-8B'
REVISION = '188130f961c8e0888a34e11121a1423c461a01ba'
CACHE = Path.home() / 'models' / 'huggingface'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image')
    parser.add_argument('caption', nargs='?', default='red cup')
    parser.add_argument('--device', default='mps')
    parser.add_argument('--runs', type=int, default=2)
    args = parser.parse_args()

    source = snapshot_download(MODEL_ID, revision=REVISION, cache_dir=str(CACHE),
                               local_files_only=True)
    t0 = time.time()
    processor = AutoProcessor.from_pretrained(
        source, trust_remote_code=True, padding_side='left', use_fast=False,
        local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(
        source, trust_remote_code=True, dtype=torch.bfloat16,
        device_map={'': args.device}, local_files_only=True)
    model.eval()
    device = next(model.parameters()).device
    print(f'Loaded on {device} in {time.time() - t0:.1f}s')

    # Same preprocessing as the ROS node: BGR frame resized to 640x480
    bgr = cv2.resize(cv2.imread(args.image), (640, 480))
    image = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    prompt = f'Point to {args.caption}'
    messages = [{'role': 'user', 'content': [
        {'type': 'text', 'text': prompt}, {'type': 'image', 'image': image}]}]

    for run in range(args.runs):
        inputs = processor.apply_chat_template(
            messages, tokenize=True, add_generation_prompt=True,
            return_tensors='pt', return_dict=True, padding=True,
            return_pointing_metadata=True)
        metadata = inputs.pop('metadata')
        inputs = {k: (v.to(device, dtype=torch.bfloat16) if torch.is_floating_point(v)
                      else v.to(device)) for k, v in inputs.items()}
        t0 = time.time()
        with torch.inference_mode():
            output = model.generate(
                **inputs,
                logits_processor=model.build_logit_processor_from_inputs(inputs),
                max_new_tokens=64, do_sample=False)
        if device.type == 'mps':
            torch.mps.synchronize()
        elapsed = time.time() - t0
        tokens = output[:, inputs['input_ids'].shape[1]:]
        text = processor.post_process_image_text_to_text(
            tokens, skip_special_tokens=False, clean_up_tokenization_spaces=False)[0]
        points = model.extract_image_points(
            text, metadata['token_pooling'], metadata['subpatch_mapping'],
            metadata['image_sizes'])
        print(f'Run {run + 1}: {elapsed:.1f}s, points={points}, text={text[:120]!r}')

    for _, _, x, y in points:
        cv2.circle(bgr, (int(x), int(y)), 9, (0, 0, 255), 2)
    out = Path(args.image).with_suffix('.molmo.jpg')
    cv2.imwrite(str(out), bgr)
    print(f'Annotated image: {out}')


if __name__ == '__main__':
    main()
