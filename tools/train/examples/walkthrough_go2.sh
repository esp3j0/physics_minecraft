#!/usr/bin/env bash
# E02j D2:go2 全链样例(从 URDF 导入身体,证换实体轻易)。详见 walkthrough.sh 头注释。
# Windows CPU:CUDA_VISIBLE_DEVICES="" bash tools/train/examples/walkthrough_go2.sh
# vm-ai GPU:PY=~/req012-gpu/.venv/bin/python BACKEND=mujoco_warp DEVICE=cuda:0 bash …
# 注:go2 mujoco 契约检查项8a base_lin_vel 为既有 backlog(生产提取正确),脚本受控断言仅该项 FAIL。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$HERE/walkthrough.sh" go2