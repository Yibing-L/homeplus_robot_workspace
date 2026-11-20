#!/usr/bin/env bash
# Needs: sudo apt-get install -y python3.10-venv
# Provision the HomePlus inference server on the AWS GPU box.
set -euxo pipefail
ROOT=$HOME/homeplus_server
mkdir -p $ROOT && cd $ROOT
[ -x venv/bin/python ] || python3 -m venv venv
. venv/bin/activate
pip install --upgrade pip wheel "setuptools>=68,<80"  # old setuptools lacks PEP 660 editable installs (sam2)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install transformers==4.57.1 'accelerate>=1.10,<2' 'bitsandbytes>=0.48,<0.50' 'einops>=0.8,<1' \
  'jinja2>=3.1,<4' decord2==3.4.0 'huggingface-hub>=0.35,<1' 'safetensors>=0.5,<1' hf_xet \
  fastapi uvicorn pillow opencv-python-headless 'numpy<2' requests
export HF_HUB_ENABLE_HF_TRANSFER=0
python - <<'PY'
from huggingface_hub import snapshot_download
p = snapshot_download(repo_id='allenai/MolmoPoint-8B',
                      revision='188130f961c8e0888a34e11121a1423c461a01ba',
                      cache_dir='/home/ubuntu/homeplus_server/models/huggingface')
print('MolmoPoint ready:', p)
PY
[ -d Grounded-SAM-2 ] || git clone --depth 1 https://github.com/IDEA-Research/Grounded-SAM-2
cd Grounded-SAM-2
SAM2_BUILD_CUDA=0 pip install --no-build-isolation -e .
mkdir -p checkpoints
[ -s checkpoints/sam2.1_hiera_tiny.pt ] || wget -q -O checkpoints/sam2.1_hiera_tiny.pt \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
pip install 'numpy<2' transformers==4.57.1
python -c "import torch,transformers,bitsandbytes,decord,sam2;print('OK',torch.__version__,transformers.__version__,torch.cuda.is_available())"
echo SETUP_DONE
