# tools/train — 四足统一训练管线(使用指南)

REQ-012 阶段 E(E02f/g/h)已把训练代码完全统一:**一份张量 PPO 循环插所有后端/设备**。
本指南面向使用者,固化"改模型 → 训练 → 导出 → 进游戏"全链,并给出"怎么改"的修改指引。
每条命令都可复制跑通(架构师会抽跑复核)。一键样例见 [`examples/`](examples/)(quad/go2 短训证通路)。

> 摆臂 demo(`train/` 包,双关节摆臂指目标点)是早期 R2/R3 单元,已精简为附录;主线是四足 `quad/` 包。

## 1. 架构总览

- **一份循环**:`quad/ppo_gpu.py::train_tensor` 是唯一训练循环;`quad/ppo.py` 的 `main()` 全路由到它。
  legacy numpy 训练循环已退役(E02h,git 历史可追溯)。
- **三个后端,同一接口**(`reset(seed)->torch.float32[n,obs_dim]`、`step(torch actions)->(obs, reward_inputs dict[str,torch], terminated, truncated)`):
  - `rapier`(CPU):Rust 物理 `agent_env.VecEnv`(PyO3),经 `NumpyToTensorAdapter` 零拷贝接入。
  - `mujoco`(CPU):MuJoCo 物理,**仍用 `mujoco.rollout` 批量推进**(不退化成逐步 mj_step/warp-cpu),经同一适配器零拷贝接入。
  - `mujoco_warp`(cpu/cuda:0/cuda:1):原生张量后端(GPU 批量物理)。
  - 适配器在 `quad/backends/tensor.py`:`torch.from_numpy` 共享内存(obs/reward_inputs/flags),actions `detach().cpu().numpy()` 转回;`make_tensor_backend(args)` 工厂统一返回张量接口。rapier/mujoco 仅 cpu,给 cuda 直接拒绝。
- **奖励双份同步**:`quad/task.py`(numpy oracle,常量 `W/SIGMA/H0/V_*_MAX`)+ `quad/reward.py`(torch `reward_gpu`,设备无关)。两者逐键加权一致,parity 测试强制 ≤1e-5。**改奖励须两处同步**(见 §4)。
- **双卡 DDP**:`quad/ppo_ddp.py`(torchrun)。每 rank 进程启动时把 `CUDA_VISIBLE_DEVICES` 重设为仅本卡并 re-exec(进程内只见一张卡),DDP allreduce 梯度、obs-norm 每步 all_gather 后按 rank 序 Chan 合并。
- **观测/解码单一事实来源在 Rust**:MuJoCo/warp 只取物理状态,观测编码、动作解码、机身系换算、意图采样、回合种子全调 `agent_env.Codec / body_frame_batch / episode_seed / sample_intent / spec_for`(PyO3)。

## 2. 环境

| | Windows(本机,CPU) | vm-ai(GPU) |
|---|---|---|
| venv | `tools/train/.venv` | `~/req012-gpu/.venv` |
| 内容 | torch-cpu / mujoco 4.0.1 / agent_env / numpy / onnx / onnxscript | torch-cuda / mujoco-warp / mujoco / agent_env / onnx(overlay) |
| Python | `.venv/Scripts/python.exe` | `~/req012-gpu/.venv/bin/python` |
| 跑法 | CPU 一律 `CUDA_VISIBLE_DEVICES=""` | `--device cuda:0`/`cuda:1`;双卡 torchrun |

- **装/重装 agent_env**(只装这一个模块进 venv,不动别的包;在仓库根):
  ```bash
  tools/train/.venv/Scripts/python.exe -m maturin develop --release -m crates/agent-env/Cargo.toml
  ```
  pyproject 在 `crates/agent-env/pyproject.toml`(`module-name="agent_env"`,`features=["python"]`)。默认工作区门禁不带 python 特性,不链接 Python。
- **零下载纪律**:pytest/onnx 等用 `uv run --no-project --python <venv-python> --offline --with <包> …` 缓存 overlay;清单外不装。**`--no-project` 必需**:`tools/train/pyproject.toml` 会让在该目录跑的 `uv run` 改用项目 `.venv`(忽略 `--python`,丢掉 agent_env)。
- **Windows 控制台编码**:torch.onnx 导出会打印 ✅ 等字符,GBK 控制台会 `UnicodeEncodeError`。**所有 Python 命令前置 `PYTHONIOENCODING=utf-8`**(样例脚本已内置)。
- 下文命令默认在 `tools/train` 下用本 venv 跑(`cd tools/train`);`cargo xtask` 在仓库根跑。vm-ai 把 `.venv/Scripts/python.exe` 换成 `~/req012-gpu/.venv/bin/python` 并按需加 `--device`。

## 3. 全链九步(改模型 → 训练 → 导出 → 进游戏)

以 quad 为例;go2 把 body/manifest 换成 go2 即可(§5 差异)。`<…>` 为占位。

### 步1 导入/改模型(URDF → body.toml)
```bash
cargo xtask import-body <model.urdf> --out <body.toml> \
    [--name <名>] [--up z|y] [--kp 100] [--kd 3] [--friction 0.6] \
    [--pose <关节>=<角> …(可重复)] [--cylinder-segments 16] [--max-hull-points 32] \
    [--package <名>=<目录> …] [--report <json>]
```
- 默认 `up=z kp=100 kd=3 friction=0.6 cylinder-segments=16 max-hull-points=32`。
- **坐标折算**:URDF z-up → 游戏 y-up,`(x,y,z)→(x,z,−y)`;URDF 旋转导入时全部折算进数据(`BodyDef` 连杆坐标系无旋转,形状只有平移 offset)。
- `--package` 映射 URDF 的 `package://` 网格路径到本地目录(网格须在 `assets/third_party/`,不下载)。
- **body_hash 是 manifest 联动键**:`body.toml` 的 `BodyHash::hex()` 必须等于 manifest 的 `body_hash`。
- quad 是手写身体(`assets/bodies/quad.body.toml`,非 URDF 导入);go2 由 `assets/third_party/unitree_go2/go2_description.urdf` 导入(选项见 `go2.body.toml` 头注释)。
- 产物:`<body.toml>`(+ 可选 report)。

### 步2 导出 MJCF(body.toml → xml)
```bash
cargo xtask export-mjcf <body.toml> --out <xml> --timestep 0.0016666666666666668 \
    [--floor-friction 0.6] [--spawn-height 0.35] [--joint-damping 0.1] [--report <json>]
```
- **timestep 必须 = 1/600 ≈ 0.0016666666666666668**:默认 0.002 不能整除 tick_hz 60,mujoco 后端会报错。
- 产物:`<xml>`(mujoco/mujoco_warp 后端的 `--mjcf`)。

### 步3 写 manifest
`quad/manifests/<名>_train.toml`:
- `body_hash` 须匹配身体(步1 的 hash);`name`;`control_period_ticks`(控制周期 = ticks × 物理 tick);
- `action_scale`/`action_clip`(动作 → 关节目标:`target = default_pose + action_scale × raw`,raw 钳 ±action_clip);
- `[[obs]]` 逐项 `term`/`scale`(顺序与维度决定 obs_dim;`default_pose` 缺省取身体各关节 default_angle)。
- quad/go2 现成 manifest:`quad_train.toml`/`go2_train.toml`(48 维,scale 全 1,action_scale 0.25,control_period_ticks 2)。

### 步4 契约检查
```bash
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe -m quad.backends.check \
    --backend rapier --body <body.toml> --manifest <man.toml>
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe -m quad.backends.check \
    --backend mujoco --body <body.toml> --manifest <man.toml> --mjcf <xml>
```
- rapier 项1–6; mujoco 项1–14(项7–14 含手算期望)。逐项 PASS/FAIL,有 FAIL 非零退出。
- 项1 逐键比对 `quad/spec.py`(独立解析)与 Rust `VecEnv.spec()`,**body_hash 不符即 FAIL**。
- **go2 mujoco 项8a base_lin_vel 是既有 backlog**(手算期望按 quad 根框钉,go2 根框取向不同;生产提取正确),仅该项 FAIL,本单不修。

### 步5 训练
```bash
# CPU(rapier 或 mujoco;一律 CVD=""):
CUDA_VISIBLE_DEVICES="" PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe -m quad.ppo \
    --backend rapier --device cpu --body <body.toml> --manifest <man.toml> \
    --n-envs 1024 --threads 8 --out <dir> --max-minutes 30
# mujoco-CPU 加 --mjcf <xml>;mujoco_warp GPU:
PYTHONIOENCODING=utf-8 ~/req012-gpu/.venv/bin/python -m quad.ppo \
    --backend mujoco_warp --device cuda:0 --body <body.toml> --manifest <man.toml> --mjcf <xml> \
    --n-envs 4096 --out <dir> --max-minutes 30
# 双卡 DDP(--n-envs 为总 N,各 rank 均分;每 rank 自动隔离单卡):
# 在 tools/train 下跑;PYTHONPATH="$PWD" 必需——每 rank re-exec 后靠它找到 quad 包。
PYTHONPATH="$PWD" torchrun --nproc_per_node=2 -m quad.ppo_ddp \
    --backend mujoco_warp --body <body.toml> --manifest <man.toml> --mjcf <xml> \
    --n-envs 4096 --out <dir> --max-minutes 30
```
- 关键选项:`--n-envs N --threads T --torch-threads --num-steps 48 --total-timesteps --max-minutes`
  `--num-minibatches 32 --update-epochs 10 --lr 3e-4 --gamma 0.99 --gae-lambda 0.95 --clip-coef 0.2`
  `--intent-range lo0 hi0 lo1 hi1 lo2 hi2 --reward-scale 0.1 --episode-controls 600 --seed`
  `--init-from <ckpt>`(续训,`spec_matches` 核对,忽略 episode_controls/fall_up_y_min)`--freeze-obs-norm`。
- **CPU 跑一律 `CUDA_VISIBLE_DEVICES=""`**(warp-cpu × CUDA 可见 → 堆损坏 abort;启动层环境变量,代码零改动)。
- 产物:`<dir>/ckpt_final.pt`(+ `ckpt_last.pt`)、`curve.csv`(iter,total_steps,ep_rew_mean,ep_len_mean,fall_frac,steps_per_sec)。
- 吞吐参考(vm-ai,稳态):mujoco-CPU@1024 ~6.8k、warp-cuda:0@1024 ~13.9k、@4096 ~48k ctrl/s;rapier-CPU 大 N 见 §6 OOM。

### 步6 导出(ckpt → 策略包)
```bash
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe -m quad.export \
    --ckpt <dir>/ckpt_final.pt --manifest <man.toml> --name <策略名> --out <export_dir> --body <body.toml>
```
- 产物:`policy.onnx`(actor 均值网络,obs float32[1,obs_dim]→action[1,act_dim],不含归一化)、
  `policy.toml`(训练 manifest 原样 + name 换 `--name` + `[normalization]` obs mean/inv_std)、
  `probe.json`(RapierBackend 1-env、intent 全 0、reset seed 0/1/2 首帧观测与动作)。
- venv 缺 onnx 时用 `uv run --no-project --python <venv> --offline --with onnx --with onnxscript python -m quad.export …`(`--no-project` 避开 `tools/train/pyproject.toml` 劫持环境)。

### 步7 游戏推理核对
```bash
cargo xtask check-policy <export_dir> --body <body.toml> --probe <export_dir>/probe.json
```
- **须在 probe 生成机跑**:跨机 obs 差 ~3.4e-5(浮点/平台),同机要求 reset 观测与 probe.obs 逐位相同、`policy.act` raw 与 probe.action 每分量差 <1e-4。
- 不带 `--probe` 只验证策略包能加载。产物:stdout 逐 probe 最大差 + OK/FAIL。

### 步8 评估
```bash
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe -m quad.eval \
    --ckpt <dir>/ckpt_final.pt --backend rapier --body <body.toml> --manifest <man.toml> \
    --intent 1 0 0 --episodes 8 --seconds 10 --seed 0
```
- `--backend mujoco` 加 `--mjcf <xml>`;跨后端 spec 相同,rapier 检查点可接 mujoco 评估。
- 产物:`<ckpt 同目录>/eval_<intent>.json`(摔倒回合数/N、存活均值、|vx_err|/|vz_err|/|yaw_err|、高度、up_y)+ stdout 摘要。

### 步9 进游戏
1. 策略包放 `assets/policies/<名>/`(`policy.toml` + `policy.onnx`;`probe.json` 可选):
   ```bash
   mkdir -p assets/policies/<名>
   cp <export_dir>/policy.toml <export_dir>/policy.onnx assets/policies/<名>/
   ```
   `policies_dir` 默认 `assets/policies`(config 可改);**body 哈希须与训练一致**。
2. 配置 `config.toml`:
   ```toml
   [[agents]]
   body = "quad"                 # go2 换 "go2"
   controller = "policy"
   policy = "<名>"
   position = [64.0, -59.5, 64.0]
   intent = [0.5, 0.0, 0.0]
   ```
   启动:`cargo run -p app -- config.toml`。
   **意图归一化上限是硬编码常量**(`crates/app/src/controller/mod.rs`:`INTENT_V_FWD_MAX=0.6` m/s、`INTENT_V_LAT_MAX=0.3` m/s、`INTENT_YAW_MAX=1.0` rad/s),不进 policy.toml。
3. 或用既有 smoke 场景(策略狗 `quad_mj1_ft` 进真实服务器 + 附身改向/转回/解除):
   ```bash
   cargo xtask smoke req012-policy-walk
   ```

## 4. 修改指引("怎么改"在哪、代价多大)

| 改什么 | 在哪 | 代价/注意 |
|---|---|---|
| **超参** | `quad/ppo.py` CLI(lr/gamma/clip/num-steps/minibatches/epochs/reward-scale…) | 零代码,命令行即可;DDP 同参 |
| **奖励** | `quad/task.py`(numpy oracle,常量 `W/SIGMA/H0/V_*_MAX`)**+** `quad/reward.py`(torch `reward_gpu`) | **两处须同步**;改后跑 `python -m quad.selftest`(手算期望 R-A/B/C)+ parity 测试(tests/test_adapter、tests_gpu/test_parity,≤1e-5) |
| **模型** | import-body → body.toml → export-mjcf → xml | manifest `body_hash` 联动;obs 维随 dof 变;重跑步1–4 |
| **obs/传感器项** | Rust `crates/agent-env`(numpy oracle Codec)**+** `quad/backends/mujoco_warp.py` `encode_gpu`/`decode_gpu`(torch)**+** 各后端状态提取 **+** manifest `[[obs]]` **+** parity 测试 | **跨 Rust/全后端**,代价大;须同步所有后端与 parity |

**当前 obs = 48 维纯本体感觉**:`base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3) + joint_pos(12) + joint_vel(12) + last_action(12) + intent(3)`。
**从运动学状态推算,非 mujoco `<sensor>`**。

- **脚底力反馈:不存在**。加它中等可行——mujoco/rapier 都能算接触力,但要动 Rust Codec + 全后端提取 + manifest + parity,跨 Rust/全后端。
- **摄像头:不存在,难**。mujoco-warp 无批量渲染器,须另起渲染管线,属独立单元(本管线不含)。
- 力反馈/摄像头均**另议**,不在当前统一管线内。

## 5. go2 与 quad 差异(换实体轻易)

| | quad | go2 |
|---|---|---|
| 来源 | 手写 body.toml(demo 定值) | URDF 导入(`unitree_go2/go2_description.urdf`) |
| body_hash | `b48ebc84674927da` | `0e1de10395e2d6aa` |
| dof / obs 维 | 12 / 48 | 12 / 48(同布局) |
| action_scale / control_period_ticks | 0.25 / 2 | 0.25 / 2 |
| manifest | `quad_train.toml` | `go2_train.toml` |
| 站立根高度(eval) | ~0.277 m | ~0.314 m(真实几何更高) |
| mujoco 契约检查 | 项1–14 全 PASS | 项8a base_lin_vel backlog(仅该项 FAIL,生产正确) |

换实体只需:步1 导入(go2 用现成 URDF)→ 步2 导出 xml → 用对应 manifest → 步4–8 照常。
一键样例:`bash tools/train/examples/walkthrough_go2.sh`(Windows CPU 前置 `CUDA_VISIBLE_DEVICES=""`)。

## 6. 已知坑(照实列)

- **warp-cpu × CUDA 可见 → 堆损坏 abort**:CPU 跑一律 `CUDA_VISIBLE_DEVICES=""`(启动层环境变量,代码零改动)。
- **check-policy 跨机 probe obs 差 ~3.4e-5**:须在 probe 生成机跑(同机逐位)。
- **rapier-CPU 大 N OOM**:VecEnv ≈14.9 MiB/env;16 GB VM n=1024 RSS ~15.2 GiB,n≥2048 SIGKILL。对照用 N=1024。
- **mujoco 4.0 无 mjx**;`make_data` 传 host `MjModel`;`put_model`/`make_data` 无 device 参数,须 `with wp.ScopedDevice(dev)` 包构造。
- **双卡同进程交替步进 cuda:0/cuda:1 → CUDA error 700**:双卡必须多进程 DDP,每进程 `CUDA_VISIBLE_DEVICES=<rank>` 隔离单卡。
- **DDP 每 rank re-exec 丢 cwd 导入**:`torchrun -m quad.ppo_ddp` 须前置 `PYTHONPATH="$PWD"`(在 tools/train 下),否则 worker 报 `No module named 'quad'`。
- **内核惯量由碰撞形状推出**(不用 `LinkDef.com`/`LinkDef.inertia`);质量生成时乘 `agent_mass_scale`(默认 0.01)。
- **go2 mujoco 项8a base_lin_vel 手算期望 backlog**(生产提取正确,本单不修)。
- **Windows GBK 控制台**:torch.onnx 打印 emoji → `UnicodeEncodeError`,前置 `PYTHONIOENCODING=utf-8`。
- **export-mjcf 默认 timestep 0.002** 不整除 tick_hz 60 → 用 `--timestep 0.0016666666666666668`(1/600)。
- **uv run 在 tools/train 下被 pyproject 劫持**:`tools/train/pyproject.toml` 使 `uv run` 改用项目 `.venv`(忽略 `--python`,丢 agent_env)→ overlay 命令一律加 `--no-project`。

## 7. 一键样例(examples/)

```bash
# Windows CPU(无 GPU):
CUDA_VISIBLE_DEVICES="" bash tools/train/examples/walkthrough_quad.sh
CUDA_VISIBLE_DEVICES="" bash tools/train/examples/walkthrough_go2.sh
# vm-ai GPU:
PY=~/req012-gpu/.venv/bin/python BACKEND=mujoco_warp DEVICE=cuda:0 bash tools/train/examples/walkthrough_quad.sh
```
串起 import(go2)→ export-mjcf → check → 短训(`--total-timesteps 50000 --max-minutes 2 --n-envs 64`)→ export → check-policy → eval。
**每步断言退出码 0 + 关键产物存在**,漏步/错路径非零退出。go2 把"仅项8a FAIL"作为受控断言。
"进游戏"步在脚本末尾以注释给出(不强制跑重型游戏构建)。

## 附录:摆臂 demo(train/ 包,早期 R2/R3)

双关节摆臂指目标点(`arm-demo`,根 fixed,两 revolute,obs 14 维,动作 2 维增量式 ±0.1)。
`train/ppo.py`(CleanRL 底本)、`train/rewards.py`、`train/export_onnx.py`、`train/backends/{rapier,check}.py`。
奖励基线 `r = −dist − 0.01·‖a_t−a_{t−1}‖² + [dist≤0.03]`,reward_scale 0.1。
契约检查:`python -m train.backends.check rapier`。该 demo 与四足统一管线独立,仅作历史参考。