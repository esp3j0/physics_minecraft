# tools/train — 后端无关训练(R2 补充;R3 修订 2026-11-08)

PPO、奖励、导出与后端解耦(R2 补充;REQ-012 §5.3 的三步路线):
1. **rapier 跑通训练**(本单元,任务 = 双关节摆臂指到目标点,R2 再补充);
2. 同一套 PyTorch 代码直接接 MuJoCo 做主训练(`transfer`,未开始);
3. 把 MuJoCo 训好的模型放回 rapier 微调(未开始)。

## 目录

```
train/
  spec.py              BackendSpec:观测布局、动作映射、控制频率、
                       reward_inputs 键、身体标识(两后端必须一致;单一
                       事实来源 = learn_env 导出常量,check.py 核对)
  rewards.py           奖励:只读 reward_inputs(+ 动作、步号)
  ppo.py               CleanRL ppo_continuous_action.py(MIT)底本改写;
                       检查点/ --init-from / --freeze-obs-norm / reward 缩放
  export_onnx.py       torch.onnx.export 导出 actor 均值网络 + policy.toml(§5.2)
  torch_export_obs.py  L3 数据:导出归一化观测 + torch 输出(learn torchtract 对照用)
  backends/
    base.py            VecBackend 接口(下一步接 MuJoCo 只需实现它)
    rapier.py          rapier 适配器:包装 learn_env.VecEnv(PyO3)
    check.py           契约检查:常量逐项/形状/dtype/键集合/reset 确定性/
                       越界钳位/到时约定/线程无关
pyproject.toml         maturin(cdylib = demos/agents/learn 的 PyO3 feature)
requirements.txt       锁定版本(torch CPU 等)
.venv/                 虚拟环境(gitignore;只装这里)
```

接 MuJoCo 时预计只需新增 `train/backends/mujoco.py`(实现 `VecBackend`,
MJCF 由 `cargo xtask export-mjcf` 导出)——`ppo.py`/`rewards.py`/`export_onnx.py`
一行不改(它们只 import `spec` 和接口;`spec_matches` 会在 `--init-from`
时核对两边的 spec 字典)。

## 环境(用户已同意的安装范围)

系统 Python 3.12 → `tools/train/.venv`;只装:
`torch==2.15.0+cpu`(index-url https://download.pytorch.org/whl/cpu)、
`numpy==2.5.4`、`gymnasium==1.4.0`、`maturin==1.16.0`、
`onnx==1.23.2` + `onnxscript`(导出 ONNX 需要)。
`learn_env` 由 `maturin develop --release` 从 demos/agents/learn 编出;
模块导出 17 个环境常量(P3),`python -m train.backends.check rapier`
逐项与 spec.py 核对。

## BackendSpec 摘要(全表见 train/spec.py)

- 身体 `arm-demo`:双关节摆臂,根 fixed,两 revolute(世界 z 轴),
  连杆 2×(0.5 m、1 kg),末端标记球 0.01 kg,限位 ±170°,无地面。
- 观测 14 维:`q1_sin, q1_cos, q2_sin, q2_cos, q1_vel, q2_vel,
  target_x, target_y, tip_x, tip_y, target_minus_tip_x, target_minus_tip_y,
  last_act1, last_act2`(相对向量 dx,dy 是策略的"待消误差")。
- 动作 2 维,取值 [−1,1](Env 内钳位,P5),**增量式**:
  目标角 += **0.1**×动作,钳限位(P3 统一;R2 的 0.05 与事实不符,删)。
- 控制:60 Hz 物理、30 Hz 控制(2 拍),回合 90 控制步 = 3 s;
  目标采样 r∈[0.3,0.9] 面积均匀、角全圆;出生角 ±0.5 rad(关节坐标
  注入,P4);无提前终止。
- PD 电机 kp 2000 / kd 60 / τ 25 N·m(选定依据见 learn README;
  L0 实证 100/100×3 种子)。

## 奖励(R3/P1 简单基线;总架构师探针第三行)

```python
REWARD_W = {"dist": 1.0, "act_rate": 0.01, "reach": 1.0, "align": 0.0, "tip_speed": 0.0}
# 每控制步:
#   r = −dist − 0.01·‖a_t − a_{t−1}‖² + 1·[dist ≤ 0.03]
# reward_scale 0.1(进 buffer 前;探针口径)。
```

R2 的 align(+max(0, v·u) 无上限)、tier 阶梯(0.3/0.1/0.03 三级)、
保持窗口 ×3、线性+tanh 叠加全部撤除(P1 主因:甩臂刷分)。R3 纪律:
在基线上每加一项都要在 README 写理由 + 加前/加后 L1 对照;
**本轮未加任何项**(s1 达标,基线够用)。权重初值 = 最终值。

## PPO(CleanRL 底本;改动都写明)

保持原样:GAE(γ λ)、clip 目标、value clip、minibatch 循环、obs Welford
归一化、正交初始化、actor/critic 双塔、**done 约定(dones[step] = obs[step]
是新回合首帧)**。
改动(全部写明):
- 环境接口:直接用 `VecBackend`(不走 gymnasium 向量包装);
- 奖励:`rewards.reward()`(如上);
- `actor_logstd` 初始 −0.5(σ≈0.6;CleanRL 默认 0 的 σ=1 在 [−1,1] 动作
  空间过半饱和,梯度 dead);
- `--reward-scale 0.1`(R3/P1 探针口径;R2 用 0.01 配旧奖励);
- `--freeze-norm-after 10`(预热 10 迭代后冻结 obs 归一化统计;
  统计滚动 → V 目标漂移 → vf_mse 失控,冻结后稳);
- 其余超参(R3/P2)= **CleanRL 默认**:num-minibatches **32**、
  update-epochs **10**(R2 的 4×4=16 梯度步/迭代太少)、gamma **0.99**、
  anneal-lr **开**、target-kl **不设**(R2 的 0.995/不开/0.05 早停全部
  偏离,已改回;λ 0.95、clip 0.2、lr 3e-4、vf_coef 0.5、ent_coef 0)。

### bug 记录(对复现有用;R2 起)

- **b_obs 未归一化**(R2 首轮发散根因):rollout 策略吃归一化 obs、更新
  minibatch 喂原始 obs → 重算 logp 全错位 → clip 0.7、KL 上百。修法 =
  `obs_buf[step] = o_norm`(CleanRL 原样本来就存归一化后的 obs)。
- **GAE done 错位**(R3/P6):R2 的 `done_buf[step]` 存"这一步之后的
  done"、`next_done` 恒 0 → 第 88 步不从第 89 步自举、第 89 步反而从
  下一回合初始状态自举。R3 改 CleanRL 原样(`dones[step] = next_done`),
  配套后端到时步返回 reset 前转移的 reward_inputs,奖励不再置 0。

## 正式训练(R2 再补充:全部报)

| 训练 | 命令(共通:`--n-envs 128 --threads 12 --torch-threads 4 --num-steps 90`;60 Hz 加 `--num-steps 180 --hz60`) | 预算内停点 |
| --- | --- | --- |
| 30 Hz 种子 0 | `--seed 0 --out out/arm_s0` | 1e7 步 / 484 s |
| 30 Hz 种子 1 | `--seed 1 --out out/arm_s1` | 1e7 步 / 486 s |
| 60 Hz 对照种子 2 | `--seed 2 --hz60 --out out/arm_s2_hz60` | 1e7 步 / 330 s |

预算 = 1e7 控制步或 30 分钟,先到为准(实际 1e7 步先到;32×10 更新下
端到端 ~20.6k 步/s,30 Hz 两跑 8 分钟出头)。R2 的试训/smoke 数据不提交。

吞吐:
- Rust 物理(纯环境,`learn throughput`):1 线程 59.0k 控制步/s
  (0.016 ms/控制周期),16 线程 453.0k(0.035 ms/控制周期);
- Python 训练端到端(收集+PPO 更新):30 Hz ~20.6k、60 Hz ~30.4k
  控制步/s(32×10 = 320 梯度步/迭代,R2 的 4×4 时是 ~37k——更新
  加重后吞吐降一半,换 s1 达标)。

## 微调路径(第 3 步的底座;R3 实测达标)

- 检查点 = actor + critic + log σ + obs 归一化统计 + 优化器状态 + spec + 超参;
- `--init-from <ckpt>`:恢复训练;`spec_matches()` 核对检查点里的 spec,
  不一致拒绝(逐字段报差异);可换后端(同 spec);
- `--freeze-obs-norm`:微调时冻结归一化统计;
- **L4 实测**(s1 → 5 分钟):末 20 迭代均值 44.69 vs 首迭代 39.12,
  差 12.46% ≤ 20%,无 NaN;
- **L5 实测**(质量 ×1.5、kp ×0.7,只改物理):零样本 185/200 →
  微调 10 分钟(21.38M 步,无 NaN)→ 变动物理 185/200、主口径 187/200;
- 导出:观测归一化参数(mean/inv_std)写进 `policy.toml`(§5.2),
  `learn eval` 在 Rust 侧读 policy.toml 做归一化再喂 tract。
