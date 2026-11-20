"""HomePlus remote inference server: MolmoPoint-8B pointing + SAM2 segmentation.

Implements parallel_prompts/CONTRACT.md. Inference code mirrors
homeplus_vision/scripts/grounding_dino_node.py (_init_molmopoint,
run_model_inference, _init_sam2, run_segmentation) so results match the robot.

Run: venv/bin/uvicorn server:app --host 127.0.0.1 --port 8765
"""
import base64
import os
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from PIL import Image as PILImage
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parent
MODEL_ID = os.environ.get('MOLMOPOINT_MODEL', 'allenai/MolmoPoint-8B')
MODEL_REVISION = os.environ.get(
    'MOLMOPOINT_REVISION', '188130f961c8e0888a34e11121a1423c461a01ba'
)
MODEL_CACHE = Path(os.environ.get('MOLMOPOINT_CACHE', ROOT / 'models/huggingface'))
GROUNDED_SAM_ROOT = Path(os.environ.get('GROUNDED_SAM_ROOT', ROOT / 'Grounded-SAM-2'))
QUANTIZATION = '4bit'
DEVICE = 'cuda:0'


class Engine:
    """Holds MolmoPoint and SAM2, loaded once. Methods copied from MolmoPointNode."""

    def __init__(self):
        self.lock = threading.Lock()
        self.model = None
        self.sam2_model = None
        self.enable_segmentation = False
        self._configure_import_paths()
        self._init_molmopoint()
        self._init_sam2()

    def _configure_import_paths(self) -> None:
        candidate_str = str(GROUNDED_SAM_ROOT)
        if GROUNDED_SAM_ROOT.is_dir() and candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)
        sam2_config_root = GROUNDED_SAM_ROOT / 'sam2'
        if sam2_config_root.is_dir():
            os.environ['HYDRA_CONFIG_PATH'] = str(sam2_config_root / 'configs')

    def _init_molmopoint(self) -> None:
        import torch
        from transformers import (
            AutoModelForImageTextToText,
            AutoProcessor,
            BitsAndBytesConfig,
        )
        from huggingface_hub import snapshot_download

        if not torch.cuda.is_available():
            raise RuntimeError('MolmoPoint-8B requires CUDA, but PyTorch cannot access a GPU.')
        requested_device = DEVICE

        checkpoint_source = snapshot_download(
            repo_id=MODEL_ID,
            revision=MODEL_REVISION,
            cache_dir=str(MODEL_CACHE),
            local_files_only=True,
        )

        model_kwargs = {
            'trust_remote_code': True,
            'dtype': torch.bfloat16,
            'local_files_only': True,
            'quantization_config': BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type='nf4',
                bnb_4bit_compute_dtype=torch.bfloat16,
                bnb_4bit_use_double_quant=True,
            ),
            'device_map': {'': requested_device},
        }
        print(f'Loading {MODEL_ID} ({QUANTIZATION}) on {requested_device} '
              f'from {checkpoint_source}', flush=True)
        self.processor = AutoProcessor.from_pretrained(
            checkpoint_source,
            trust_remote_code=True,
            padding_side='left',
            use_fast=False,
            local_files_only=True,
        )
        self.model = AutoModelForImageTextToText.from_pretrained(
            checkpoint_source,
            **model_kwargs,
        )
        self.model.eval()
        self.torch = torch
        self.model_device = next(self.model.parameters()).device
        self.inference_dtype = torch.bfloat16

    def _init_sam2(self) -> None:
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as exc:
            print(f'SAM2 import failed; disabling segmentation: {exc}', flush=True)
            return

        # Contract: sam2.1_hiera_tiny, falling back to sam2_hiera_tiny. Config
        # and checkpoint are paired so the architecture always matches.
        pairs = [
            (GROUNDED_SAM_ROOT / 'sam2/configs/sam2.1/sam2.1_hiera_t.yaml',
             GROUNDED_SAM_ROOT / 'checkpoints/sam2.1_hiera_tiny.pt'),
            (GROUNDED_SAM_ROOT / 'sam2/configs/sam2/sam2_hiera_t.yaml',
             GROUNDED_SAM_ROOT / 'checkpoints/sam2_hiera_tiny.pt'),
        ]
        found = next(((c, k) for c, k in pairs if c.is_file() and k.is_file()), None)
        if found is None:
            print('SAM2 config/checkpoint not found; segmentation disabled.', flush=True)
            return
        sam2_config, sam2_checkpoint = found

        sam2_config_name = str(sam2_config)
        sam2_package_root = GROUNDED_SAM_ROOT / 'sam2'
        try:
            sam2_config_name = sam2_config.relative_to(sam2_package_root).as_posix()
        except ValueError:
            pass

        sam2_model = build_sam2(
            sam2_config_name,
            str(sam2_checkpoint),
            device=str(self.model_device),
        )
        self.sam2_model = SAM2ImagePredictor(sam2_model)
        self.sam2_name = sam2_checkpoint.name
        self.enable_segmentation = True
        print(f'SAM2 loaded: {sam2_config_name} + {sam2_checkpoint}', flush=True)

    def run_model_inference(self, image_np, caption, max_new_tokens):
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_pil = PILImage.fromarray(image_rgb)
        prompt = caption.strip()
        if not prompt.lower().startswith(('point ', 'locate ', 'show ')):
            prompt = f'Point to {prompt}'
        messages = [{
            'role': 'user',
            'content': [
                {'type': 'text', 'text': prompt},
                {'type': 'image', 'image': image_pil},
            ],
        }]
        model_inputs = self.processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_tensors='pt',
            return_dict=True,
            padding=True,
            return_pointing_metadata=True,
        )
        metadata = model_inputs.pop('metadata')
        model_inputs = {
            key: (
                value.to(self.model_device, dtype=self.inference_dtype)
                if self.torch.is_floating_point(value)
                else value.to(self.model_device)
            )
            for key, value in model_inputs.items()
        }

        autocast_enabled = self.model_device.type == 'cuda'
        with self.torch.inference_mode(), self.torch.autocast(
            device_type=self.model_device.type,
            dtype=self.inference_dtype,
            enabled=autocast_enabled,
        ):
            output = self.model.generate(
                **model_inputs,
                logits_processor=self.model.build_logit_processor_from_inputs(
                    model_inputs
                ),
                max_new_tokens=max_new_tokens,
                do_sample=False,
            )
        generated_tokens = output[:, model_inputs['input_ids'].shape[1]:]
        if hasattr(self.processor, 'post_process_image_text_to_text'):
            generated_text = self.processor.post_process_image_text_to_text(
                generated_tokens,
                skip_special_tokens=False,
                clean_up_tokenization_spaces=False,
            )[0]
        else:
            generated_text = self.processor.decode(
                generated_tokens[0],
                skip_special_tokens=False,
                clean_up_tokenization_spaces=False,
            )
        points = self.model.extract_image_points(
            generated_text,
            metadata['token_pooling'],
            metadata['subpatch_mapping'],
            metadata['image_sizes'],
        )
        # Raw points; selection/clipping stays on the client like the node.
        raw = [
            [int(object_id), int(image_index), float(x), float(y)]
            for object_id, image_index, x, y in points
        ]
        return prompt, generated_text, raw

    def run_segmentation(self, image_np, point):
        if not self.enable_segmentation:
            return [], []
        image_rgb = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        self.sam2_model.set_image(image_rgb)
        masks, scores, _ = self.sam2_model.predict(
            point_coords=np.asarray([point], dtype=np.float32),
            point_labels=np.asarray([1], dtype=np.int32),
            multimask_output=True,
        )
        if masks is None or len(masks) == 0:
            return [], []
        best = int(np.argmax(scores))
        return masks[best:best + 1], scores[best:best + 1]


def decode_jpeg(b64: str) -> np.ndarray:
    try:
        buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    except Exception as exc:
        raise HTTPException(400, f'bad base64: {exc}')
    image = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # BGR, as the node receives
    if image is None:
        raise HTTPException(400, 'could not decode JPEG')
    return image


class PointRequest(BaseModel):
    image_jpeg_b64: str
    caption: str
    max_new_tokens: int = 64


class SegmentRequest(BaseModel):
    image_jpeg_b64: str
    point: List[float]


app = FastAPI(title='HomePlus inference')
engine: Optional[Engine] = None


@app.on_event('startup')
def _startup():
    global engine
    engine = Engine()


@app.get('/health')
def health():
    return {
        'ok': engine is not None and engine.model is not None,
        'molmopoint': engine is not None and engine.model is not None,
        'sam2': engine is not None and engine.enable_segmentation,
        'device': str(engine.model_device) if engine else DEVICE,
        'quantization': 'bnb-4bit-nf4-double-quant/bf16-compute',
    }


@app.post('/point')
def point(req: PointRequest):
    if not req.caption.strip():
        raise HTTPException(400, 'caption must be nonempty text')
    image = decode_jpeg(req.image_jpeg_b64)
    h, w = image.shape[:2]
    # Same floor as the node's max_new_tokens parameter.
    max_new_tokens = max(8, int(req.max_new_tokens))
    with engine.lock:
        t0 = time.perf_counter()
        prompt, generated_text, points = engine.run_model_inference(
            image, req.caption, max_new_tokens
        )
        elapsed = time.perf_counter() - t0
    print(f'/point prompt="{prompt}" points={points} {elapsed:.3f}s '
          f'text={generated_text[:200]!r}', flush=True)
    return {
        'prompt': prompt,
        'generated_text': generated_text,
        'points': points,
        'image_size': [w, h],
        'elapsed_sec': elapsed,
    }


@app.post('/segment')
def segment(req: SegmentRequest):
    if len(req.point) != 2:
        raise HTTPException(400, 'point must be [u, v]')
    image = decode_jpeg(req.image_jpeg_b64)
    with engine.lock:
        t0 = time.perf_counter()
        masks, scores = engine.run_segmentation(image, req.point)
        elapsed = time.perf_counter() - t0
    if len(masks) == 0:
        return {'mask_png_b64': None, 'score': None, 'elapsed_sec': elapsed}
    mask = (np.asarray(masks[0]) > 0).astype(np.uint8) * 255
    ok, png = cv2.imencode('.png', mask)
    if not ok:
        raise HTTPException(500, 'PNG encode failed')
    return {
        'mask_png_b64': base64.b64encode(png.tobytes()).decode('ascii'),
        'score': float(scores[0]),
        'elapsed_sec': elapsed,
    }
