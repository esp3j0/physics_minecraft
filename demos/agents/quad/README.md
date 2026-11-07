# `quad` —— REQ-012 A00 demo 公共代码(A00-2 抽取)

Go2 尺度四足模型、足端逆解、开环步态、运行时电机目标(`set_target`)、
阻尼缩放的公共 lib。独立 crate(空 `[workspace]`),`rapier3d` 锁
`=0.36.1`,**不进工作区**;`gait` / `terrain` / `mass` 用路径依赖引用。

## 抽取纪律(A00-2 放行)

抽完后 `gait` 全矩阵 **134 行状态哈希与 9a7405a 提交数据逐行相同**
(2026-11-06 复核通过):模型、逆解、步态、电机写入的数值路径与
9a7405a 的 `demos/agents/gait/src/main.rs` 逐行一致,只改 pub。
相位滞后、力矩饱和等**输出列**按 A00-1 验收"要改的四处"修正过
(见 gait README),不属于"抽取改行为"。

## 内容

- 模型常量:腿 0.213×2 m、足球 r 0.022、髋 (±0.1934, 0, ±0.0465)、
  横向偏置 0.0955;质量 6.921/0.678/1.152/0.154/0.04 ≈ 15 kg;
- `spawn_robot` / `spawn_robot_yawed`:碰撞体 `.mass()`、全部关节
  `contacts_enabled(false)`、脚 = shank 末端独立 ball;yawed 版把全部
  刚体出生位姿绕世界 Y 旋转(revolute 轴是双方局部系,关节局部系跟随,
  用于 terrain 的航向偏置/斜穿);
- `set_target`:运行时经 `link.joint.data.motors` 写 ForceBased PD +
  力矩上限,目标变化时 `wake_up` 整个 multibody(不重建);
- `ik` / `Gait::foot_target`:足端轨迹(支撑匀速后扫 + 摆动 smoothstep +
  正弦抬脚)+ 2 连杆逆解(膝朝后);
- `DampMode`(Scaled/Unscaled/Zero)+ `production_params`:
  rapier 默认参数只覆写 dt;默认阻尼 0.1 是绝对常量,质量缩放时同步 ×s;
- `default_gait` / `DEFAULT_KP/KD`:A00-2 放行定的默认步态组
  (kp100 kd3 f3 s0.15 l0.06 脚摩擦 0.6,30 Hz 目标更新);
- `set_extra_solver_iterations`(A00-2 R2):对主体全部连杆+机身调
  `RigidBody::set_additional_solver_iterations(n)`;求解按连通分量取最大值,
  整个分量多跑 n 个 solver substep,`dt` 不变(不碰 D4);`n = 0` 不调用
  任何 API(数值路径不变)。

## 三个 rapier 坑(生产实现要避开;详见 gait README §13)

1. revolute 的电机/限位恒在 `AngX` 槽(对 AngZ 写 = 约束不建);
2. island 睡眠后电机约束不被求解——目标变化须 `wake_up`;
3. 关节默认阻尼是绝对常量 0.1,不随质量缩放(×0.01 档不缩放则 vfwd
   0.53 → 0.088)。
