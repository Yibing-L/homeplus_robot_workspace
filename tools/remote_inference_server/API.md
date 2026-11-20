# Remote inference contract (shared by Track B server and Track D client)

Server: FastAPI/uvicorn on the AWS GPU box, bound to 127.0.0.1:8765 only.
Reached from the VM as http://192.168.64.1:8765 via a Mac-side tunnel:
  ssh -i <your-aws-key.pem> -N -L 192.168.64.1:8765:127.0.0.1:8765 ubuntu@<AWS_IP>

GET  /health
  -> {"ok": true, "molmopoint": bool, "sam2": bool, "device": "cuda:0", "quantization": "..."}

POST /point
  body: {"image_jpeg_b64": str (640x480 BGR frame encoded as JPEG, quality 95),
         "caption": str (raw caption, e.g. "red cup"; server adds "Point to " exactly like
                         MolmoPointNode.run_model_inference),
         "max_new_tokens": int (default 64)}
  -> {"prompt": str, "generated_text": str,
      "points": [[object_id, image_index, x, y], ...]   # raw model.extract_image_points output, pixel coords in the 640x480 image
      "image_size": [w, h], "elapsed_sec": float}

POST /segment
  body: {"image_jpeg_b64": str (same 640x480 frame), "point": [u, v]}
  -> {"mask_png_b64": str | null (mono8 PNG, 255 = object, same size as image; best mask by score, multimask_output=True),
      "score": float | null, "elapsed_sec": float}

Model settings must match the robot exactly: allenai/MolmoPoint-8B at revision
188130f961c8e0888a34e11121a1423c461a01ba, 4-bit NF4 (BitsAndBytesConfig as in
homeplus_vision/scripts/grounding_dino_node.py _init_molmopoint), bf16 compute,
do_sample=False. SAM2: sam2.1_hiera_tiny (falls back to sam2_hiera_tiny).
