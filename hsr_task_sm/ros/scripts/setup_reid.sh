#!/bin/bash
# Install FastReID (torchreid) on slave laptop with RTX 3070
# Run once on the laptop.

set -e
echo "=== Installing ReID dependencies for RTX 3070 ==="

# PyTorch with CUDA 11.8
echo "Installing PyTorch CUDA 11.8..."
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# OpenCV + numpy
pip3 install opencv-python numpy

# torchreid (OSNet — fast and accurate for person re-id)
echo "Installing torchreid..."
pip3 install torchreid

echo ""
echo "=== Verifying GPU ==="
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none')"

echo ""
echo "=== Done. Run the server with: ==="
echo "   rosrun hsr_task_sm reid_server.py"
