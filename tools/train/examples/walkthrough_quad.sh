#!/usr/bin/env bash
# E02j D2:quad 全链样例(手写 body.toml)。详见 walkthrough.sh 头注释。
# Windows CPU:CUDA_VISIBLE_DEVICES="" bash tools/train/examples/walkthrough_quad.sh
# vm-ai GPU:PY=~/req012-gpu/.venv/bin/python BACKEND=mujoco_warp DEVICE=cuda:0 bash …
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/walkthrough.sh" quad