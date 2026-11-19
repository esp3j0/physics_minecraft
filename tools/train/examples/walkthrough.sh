#!/usr/bin/env bash
# E02j D2:四足训练全链一键样例(证通路,不求效果)。
#
# 用法(在仓库根或任意目录均可,脚本自定位):
#   bash tools/train/examples/walkthrough.sh quad      # 手写 quad 身体
#   bash tools/train/examples/walkthrough.sh go2       # 从 URDF 导入 go2
# 等价薄包装:walkthrough_quad.sh / walkthrough_go2.sh。
#
# 串起:导入(go2)→ export-mjcf → 契约检查 → 短训 → 导出 → check-policy → eval。
# 每步断言退出码 0 + 关键产物存在;漏步/错路径即非零退出(set -euo pipefail)。
# 短训预算固定 --total-timesteps 50000 --max-minutes 2 --n-envs 64(只证管线能跑)。
#
# 可选环境变量:
#   PY       Python 解释器。默认 Windows venv:tools/train/.venv/Scripts/python.exe。
#   BACKEND  训练后端 rapier|mujoco|mujoco_warp(默认 rapier)。
#   DEVICE   cpu|cuda:0|cuda:1(默认 cpu;rapier/mujoco 仅 cpu)。
#
# ── Windows CPU 跑法(无 GPU)──
#   CUDA_VISIBLE_DEVICES="" bash tools/train/examples/walkthrough_quad.sh
#   (CPU 跑一律 CVD="":warp-cpu × CUDA 可见会堆损坏;rapier/mujoco 亦不受影响。)
#   .venv 已含 torch-cpu/mujoco/agent_env/onnx,直接可跑;agent_env 重装见 README。
#
# ── vm-ai GPU 跑法 ──
#   PY=~/req012-gpu/.venv/bin/python BACKEND=mujoco_warp DEVICE=cuda:0 \
#       bash tools/train/examples/walkthrough_quad.sh
#   双卡 DDP 不走本脚本(单进程),见 README:
#     torchrun --nproc_per_node=2 -m quad.ppo_ddp … --n-envs <总N>
#   若 GPU venv 未装 onnx,导出步自动改用 uv 离线 overlay(--with onnx --with onnxscript,零下载)。
#
# 已知 backlog(本单不修):go2 的 mujoco 契约检查项8a base_lin_vel 手算期望 (0,0,1)
#   是按 quad 根坐标系钉的,go2 根框取向不同 → 实得 (0,~0,0.99626),仅该项 FAIL;
#   生产提取正确。go2 脚本把"仅 8a FAIL"作为受控断言,其余项须全 PASS。
set -euo pipefail

# Windows 控制台默认 GBK:torch.onnx 导出会打印 ✅ 等字符,须 utf-8 否则 UnicodeEncodeError。
export PYTHONIOENCODING=utf-8

BODYNAME="${1:-quad}"
case "$BODYNAME" in quad|go2) ;; *) echo "用法:walkthrough.sh {quad|go2}" >&2; exit 2;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAIN="$(cd "$HERE/.." && pwd)"
ROOT="$(cd "$TRAIN/../.." && pwd)"
PY="${PY:-$TRAIN/.venv/Scripts/python.exe}"
BACKEND="${BACKEND:-rapier}"
DEVICE="${DEVICE:-cpu}"
OUT="$ROOT/target/e02j/$BODYNAME"
mkdir -p "$OUT"

# CPU 跑一律隐藏 CUDA(warp-cpu × CUDA 堆损坏规避);GPU 跑保留可见设备。
if [ "$DEVICE" = "cpu" ]; then export CUDA_VISIBLE_DEVICES=""; fi

MAN="$TRAIN/quad/manifests/${BODYNAME}_train.toml"
MJCF="$OUT/$BODYNAME.xml"

# 导出步解释器:venv 缺 onnx 时退回 uv 离线 overlay(零下载)。
# --no-project 必需:tools/train/pyproject.toml 会让 uv run 改用项目 .venv(丢 agent_env)。
EXPORT_CMD=("$PY")
if ! "$PY" -c "import onnx, onnxscript" >/dev/null 2>&1; then
  EXPORT_CMD=(uv run --no-project --python "$PY" --offline --with onnx --with onnxscript python)
fi

say() { printf '\n==== %s ====\n' "$1"; }
need() { test -e "$1" || { echo "缺产物:$1" >&2; exit 1; }; }

# ── 步1 导入/改模型 ──
if [ "$BODYNAME" = "go2" ]; then
  say "步1 import-body(go2 URDF → body.toml)"
  BODY="$OUT/go2.body.toml"
  (cd "$ROOT" && cargo xtask import-body \
      assets/third_party/unitree_go2/go2_description.urdf \
      --out "$BODY" --name go2 --up z --kp 100 --kd 3 --friction 0.6 \
      --cylinder-segments 16 --max-hull-points 32 \
      --pose FL_thigh_joint=0.8 --pose FL_calf_joint=-1.5 \
      --pose FR_thigh_joint=0.8 --pose FR_calf_joint=-1.5 \
      --pose RL_thigh_joint=0.8 --pose RL_calf_joint=-1.5 \
      --pose RR_thigh_joint=0.8 --pose RR_calf_joint=-1.5)
  need "$BODY"
else
  say "步1 身体(quad 用现成手写 body.toml;改模型见 README import-body)"
  BODY="$ROOT/assets/bodies/quad.body.toml"
  need "$BODY"
fi
need "$MAN"

# ── 步2 导出 MJCF(timestep=1/600,整除 tick_hz 60)──
say "步2 export-mjcf"
(cd "$ROOT" && cargo xtask export-mjcf "$BODY" --out "$MJCF" --timestep 0.0016666666666666668)
need "$MJCF"

# ── 步3 契约检查(rapier 项1–6;mujoco 项1–14)──
say "步3a check rapier(项1–6)"
(cd "$TRAIN" && "$PY" -m quad.backends.check --backend rapier --body "$BODY" --manifest "$MAN")

say "步3b check mujoco(项1–14)"
if [ "$BODYNAME" = "quad" ]; then
  (cd "$TRAIN" && "$PY" -m quad.backends.check --backend mujoco \
      --body "$BODY" --manifest "$MAN" --mjcf "$MJCF")
else
  # go2:仅项8a base_lin_vel 为既有 backlog(生产提取正确),受控断言"恰 1 项 FAIL 且为 8a"。
  set +e
  mjout="$(cd "$TRAIN" && "$PY" -m quad.backends.check --backend mujoco \
      --body "$BODY" --manifest "$MAN" --mjcf "$MJCF" 2>&1)"
  rc=$?
  set -e
  printf '%s\n' "$mjout" | tail -3
  nfail="$(printf '%s\n' "$mjout" | grep -c '\[FAIL\]' || true)"
  if [ "$rc" -eq 0 ] || [ "$nfail" -ne 1 ] || ! printf '%s\n' "$mjout" | grep -q '8a base_lin_vel'; then
    echo "go2 mujoco 检查偏离已知 backlog(应恰 1 项 FAIL=8a base_lin_vel,rc=$rc,nfail=$nfail)" >&2
    exit 1
  fi
  echo "go2 mujoco:仅项8a base_lin_vel FAIL(既有 backlog,生产提取正确,本单不修),其余全 PASS。"
fi

# ── 步4 短训(证通路)──
say "步4 短训 backend=$BACKEND device=$DEVICE"
train_extra=()
if [ "$BACKEND" != "rapier" ]; then train_extra+=(--mjcf "$MJCF"); fi
(cd "$TRAIN" && "$PY" -m quad.ppo --backend "$BACKEND" --device "$DEVICE" \
    --body "$BODY" --manifest "$MAN" "${train_extra[@]}" \
    --n-envs 64 --threads 4 --torch-threads 2 --num-steps 48 \
    --total-timesteps 50000 --max-minutes 2 --num-minibatches 4 --update-epochs 2 \
    --report-every 5 --intent-range -1 1 -1 1 -1 1 --seed 0 --out "$OUT/train")
need "$OUT/train/ckpt_final.pt"
need "$OUT/train/curve.csv"

# ── 步5 导出 ONNX + policy.toml + probe.json ──
say "步5 export"
(cd "$TRAIN" && "${EXPORT_CMD[@]}" -m quad.export --ckpt "$OUT/train/ckpt_final.pt" \
    --manifest "$MAN" --name "${BODYNAME}_walk" --out "$OUT/export" --body "$BODY")
need "$OUT/export/policy.onnx"
need "$OUT/export/policy.toml"
need "$OUT/export/probe.json"

# ── 步6 游戏推理核对(须在 probe 生成机跑;跨机 obs 差 ~3.4e-5)──
say "步6 check-policy(同机)"
(cd "$ROOT" && cargo xtask check-policy "$OUT/export" --body "$BODY" --probe "$OUT/export/probe.json")

# ── 步7 评估(rapier 均值动作;跨后端 spec 相同)──
say "步7 eval(rapier,intent 1 0 0,8×10s)"
(cd "$TRAIN" && "$PY" -m quad.eval --ckpt "$OUT/train/ckpt_final.pt" --backend rapier \
    --body "$BODY" --manifest "$MAN" --intent 1 0 0 --episodes 8 --seconds 10 --seed 0)
need "$OUT/train/eval_1_0_0.json"

say "全链 PASS($BODYNAME,backend=$BACKEND,device=$DEVICE)"
echo "产物目录:$OUT"

# ── 步8 进游戏(注释,不强制跑重型游戏构建)──
# 1) 策略包放入 assets/policies/<名>/(policy.toml + policy.onnx;probe.json 可选):
#      mkdir -p assets/policies/${BODYNAME}_walk
#      cp "$OUT/export/policy.toml" "$OUT/export/policy.onnx" assets/policies/${BODYNAME}_walk/
#    (body 哈希须与训练一致;policies_dir 默认 assets/policies,可在 config 改。)
# 2) 配置 config.toml 写:
#      [[agents]]
#      body = "quad"            # go2 换 "go2"
#      controller = "policy"
#      policy = "${BODYNAME}_walk"
#      position = [64.0, -59.5, 64.0]
#      intent = [0.5, 0.0, 0.0]  # 归一化上限硬编码 controller/mod.rs:V_FWD 0.6/V_LAT 0.3/YAW 1.0
#    启动:cargo run -p app -- config.toml
# 3) 或用既有 smoke 场景(策略狗 quad_mj1_ft 进真实服务器 + 附身改向):
#      cargo xtask smoke req012-policy-walk