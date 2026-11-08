# tools/train — 后端无关训练(R2 补充,2026-11-08)

PPO、奖励、导出与后端解耦(R2 补充;REQ-012 §5.3 的三步路线):
1. **rapier 跑通训练**(本单元,任务 = 双关节摆臂指到目标点,R2 再补充);
2. 同一套 PyTorch 代码直接接 MuJoCo 做主训练(`transfer`,未开始);
3. 把 MuJoCo 训好的模型放回 rapier 微调(未开始)。

## 目录

```
train/
  spec.py              BackendSpec:观测布局、动作映射、控制频率、
                       reward_inputs 键、身体标识(两后端必须一致;单一事实来源)
  rewards.py           奖励:只读 reward_inputs(+ 动作、步号)
  ppo.py               CleanRL ppo_continuous_action.py(MIT)底本改写;
                       检查点/ --init-from / --freeze-obs-norm / reward 缩放
  export_onnx.py       torch.onnx.export 导出 actor 均值网络 + policy.toml(§5.2)
  torch_export_obs.py  L3 数据:导出归一化观测 + torch 输出(learn torchtract 对照用)
  backends/
    base.py            VecBackend 接口(下一步接 MuJoCo 只需实现它)
    rapier.py          rapier 适配器:包装 learn_env.VecEnv(PyO3)
    check.py           契约检查:形状/dtype/键集合/reset 确定性/越界钳位/线程无关
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
`onnx==1.23.2` + `onnxscript`(导出 ONNX 需要,R2 放行"必要时装 onnx")。
`learn_env` 由 `maturin develop --release` 从 demos/agents/learn 编出。

## BackendSpec 摘要(全表见 train/spec.py)

- 身体 `arm-demo`:双关节摆臂,根 fixed,两 revolute(世界 z 轴),
  连杆 2×(0.5 m、1 kg),末端标记球 0.01 kg,限位 ±170°,无地面。
- 观测 14 维:`q1_sin, q1_cos, q2_sin, q2_cos, q1_vel, q2_vel,
  target_x, target_y, tip_x, tip_y, target_minus_tip_x, target_minus_tip_y,
  last_act1, last_act2`(相对向量 dx,dy 是策略的"待消误差";
  14 维布局两后端一致)。
- 动作 2 维,取值 [−1,1],**增量式**:目标角 += 0.05×动作,钳限位
  (0.1 rad/步过冲重,0.05 稳;README 记录)。
- 控制:60 Hz 物理、30 Hz 控制(2 拍),回合 90 控制步 = 3 s;
  目标采样 r∈[0.3,0.9] 面积均匀、角全圆;出生角 ±0.5 rad;无提前终止。
- PD 电机 kp 2000 / kd 60 / τ 25 N·m(选定依据见 learn README;
  L0 实证 100/100)。

## 奖励(权重初值 = 最终值;调参过程见 learn README 的冒烟记录)

```python
REWARD_W = {"dist": 1.0, "act_rate": 0.01, "reach": 2.0, "align": 1.0, "tip_speed": 0.0}
# 每控制步:
#   −(0.5·dist + tanh(4·dist))·hold        距离:线性(远处恒梯度)+tanh(近处陡)
#   −0.01·‖a_t − a_{t−1}‖²                 动作变化
#   +2.0·hold·tier                          分级接近:≤0.3 m +0.3 / ≤0.1 m +0.7 / ≤0.03 m +1.0
#   +1.0·max(0, tip_v·u)·(未到时)          末端速度对目标方向的对齐
# hold = 3 当步号 ≥ 75(最后 0.5 s = L1 保持窗口,加权对齐判据),否则 1。
```

## PPO(CleanRL 底本;改动都写明)

保持原样:GAE(γ λ)、clip 目标、value clip、minibatch 循环、
obs Welford 归一化、正交初始化、actor/critic 双塔。
改动(全部写明):
- 环境接口:直接用 `VecBackend`(不走 gymnasium 向量包装);
- 奖励:`rewards.reward()`(如上);
- `actor_logstd` 初始 −0.5(σ≈0.6;CleanRL 默认 0 的 σ=1 在 [−1,1] 动作
  空间过半饱和,梯度 dead);
- `--reward-scale 0.01`(回合 return ~−100 → critic 可拟合;legged_gym 惯例);
- `--freeze-norm-after 10`(预热 10 迭代后冻结 obs 归一化统计;
  统计滚动 → V 目标漂移 → vf_mse 失控,冻结后稳);
- `--target-kl 0.05` 早停;`--anneal-lr 0`(anneal 到 0 后策略不再更新)。
- 超参:γ 0.995(90 步回合,γ^75=0.47 是"先走过去"学不出的原因之一;
  0.995^75=0.69)、λ 0.95、clip 0.2、5 epoch→**4 epoch**、minibatch 4、
  lr 3e-4、vf_coef 0.5、ent_coef 0。

### bug 记录(对复现有用)

- **b_obs 未归一化**(首轮 1e7 步训练发散的根因):rollout 时策略吃
  归一化 obs,更新时 minibatch 却喂原始 obs → 重算 logp 与旧 logp 完全
  错位 → clip 0.7、KL 上百。修法 = `obs_buf[step] = o_norm`
  (CleanRL 原样本来就是存归一化后的 obs,是改写时引入的错)。

## 正式训练(R2 再补充:全部报)

| 训练 | 命令(共通:`--n-envs 128 --threads 12 --torch-threads 4 --num-steps 90 --num-minibatches 4 --gamma 0.995 --target-kl 0.05 --anneal-lr 0`) | 预算内停点 |
| --- | --- | --- |
| 30 Hz 种子 0 | `--seed 0 --out out/arm_s0` | 1e7 步 / 269 s |
| 30 Hz 种子 1 | `--seed 1 --out out/arm_s1` | 1e7 步 / 272 s |
| 60 Hz 对照种子 2 | `--seed 2 --hz60 --out out/arm_s2_hz60 --num-steps 180` | 1e7 步 / 246 s |

预算 = 1e7 控制步或 30 分钟,先到为准(实际 1e7 步 ~4.5 分钟先到;
墙钟上限从未绑定)。**修正前的试训数据不提交**(诊断过程的 smoke 目录
已删;结论摘要进 learn README)。

吞吐:
- Rust 物理(纯环境,`learn throughput`):1 线程 45.6k 控制步/s
  (0.021 ms/控制周期),16 线程 304.7k(0.052 ms/控制周期)——
  物理/控制步比四足(gait 探针 ~0.15 ms/拍)便宜,双关节链短;
- Python 训练端到端(收集+PPO 更新):~37k 控制步/s(三种子一致);
  物理:网络 ≈ 10:1(0.021 ms 物理 vs ~0.027 ms 网络/控制周期,128 env
  摊薄后网络前向+反向占 ~55%)。

## 微调路径(第 3 步的底座,本单元已验证)

- 检查点 = actor + critic + log σ + obs 归一化统计 + 优化器状态 + spec + 超参;
- `--init-from <ckpt>`:恢复训练;`spec_matches()` 核对检查点里的 spec,
  不一致拒绝(逐字段报差异);可换后端(同 spec);
- `--freeze-obs-norm`:微调时冻结归一化统计;
- 导出:观测归一化参数(mean/inv_std)写进 `policy.toml`(§5.2),
  `learn eval` 在 Rust 侧读 policy.toml 做归一化再喂 tract
  (当前 eval 直接读 toml 同目录约定;归一化实现于 learn/src/main.rs)。
