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
- `set_extra_solver_iterations`(A00-2 R2 引入;**R3 已判不采用**):
  对主体全部连杆+机身调 `RigidBody::set_additional_solver_iterations(n)`;
  R2 隔离性实验证实——世界里只要有一个 multibody,rapier 0.36.1 就把
  全部醒着的刚体合成一组求解(`staged_island_solver/init.rs:62` 的
  `multi_group`),extra 实际等同改全局迭代数,改变同一分片内无关刚体
  的数值路径(20 m 外孤立堆哈希因此改变,见 mass README 隔离性一节),
  不是按主体隔离。**函数保留只用于实验,生产不采用**;`n = 0` 不调用
  任何 API(数值路径不变)。

## 三个 rapier 坑(生产实现要避开;详见 gait README §13)

1. revolute 的电机/限位恒在 `AngX` 槽(对 AngZ 写 = 约束不建);
2. island 睡眠后电机约束不被求解——目标变化须 `wake_up`;
3. 关节默认阻尼是绝对常量 0.1,不随质量缩放(×0.01 档不缩放则 vfwd
   0.53 → 0.088)。

## A00-3 追加(2026-11-07):快照 / 恢复 API

`migrate` / `freeze` 两个 demo 共用的追加(放行:quad 只允许追加,追加后
`gait` 168 行、`terrain` 85 行、`mass` 5313 行状态哈希与提交数据逐位相同,
重跑比对已验证):

- `AgentSnapshot`:迁移信封(根位姿、根速度、关节坐标、关节速度、目标;
  另带连杆位姿作判据 1 的核对基准);`snapshot_robot(world, robot, targets,
  full)` 读取,`full = false` 时速度字段清零("不带速度的快照"对照);
- `restore_robot(world, snap, ...)`:fresh `spawn_robot_yawed` + 写回。
  坐标恢复的唯一公开路径 = `Multibody::apply_displacements`(增量位移;
  `coords` 是 `pub(crate)`,revolute 等价 `coords[AngX] += disp` 并重建
  `joint_rot`)+ `forward_kinematics(bodies, true)`(从根刚体位姿重算全部
  连杆)+ `update_rigid_bodies`(位姿写回刚体);广义速度走
  `generalized_velocity_mut`,根刚体 `linvel/angvel` 同时直写;最后整只
  `wake_up(true)`(生产"目标变化即唤醒"语义);
- `measure_restore_error`:重建后、步进前的误差核对(关节坐标 rad /
  连杆位置 m / 连杆转角 rad / 速度)。关节速度读
  `Multibody::generalized_velocity()`(`link.joint_velocity` 是 solver
  每拍回写的字段,步进前恒 0,不适用)。

实测:完整快照重建误差**逐位为 0**(migrate 全矩阵 97 个 full 行、
freeze snap 全部 21 行)。
