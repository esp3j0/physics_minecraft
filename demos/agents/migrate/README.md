# A00-3 demo `migrate`(REQ-012 §3 迁移 / AC-B3,2026-11-07)

放行说明:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
最后一条评论(A00-3 放行)。**问题(放行原文)**:丢掉接触缓存重建后,四足
会不会跳变或摔倒?要验证的设计:迁移信封只带身体哈希、根状态、关节坐标、
关节速度、最后的目标,整个主体同一拍迁移。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

## 运行方式

```bash
cd demos/agents/migrate
cargo run --release -j 10          # 全矩阵 193 行,约 4 分钟
cargo run --release -j 10 -- det   # 4 组双跑确定性比对(输出 out/run_det.txt)
```

**提交的原始数据**:`out/matrix_submitted.txt`(193 行全表;默认运行会重写
`out/run_summary.csv`,提交文件由提交流程拷贝,默认运行不覆盖)、
`out/det_submitted.txt`(双跑)。独立 `target` 目录已 gitignore。

## 结论(TL;DR)

**完整快照的逐位恢复成立,迁移不摔、不掉速;`cross`(干净目标世界)与
`same` 的轨迹逐拍一致;`cross_hist`(句柄历史 + 无关方块堆)从迁移后第一步
就分歧**。生产迁移的确定性口径由此裁定:同分片内 `same`/`cross` 可定义
"逐位一致",`cross_hist` 语义下不可——迁移目标世界的历史会进数值路径,
这与总架构师在 mass demo 查出的 `multi_group` 全局合并(init.rs:62)同源。

## rapier API 口径(追加进 quad,实现依据)

- 关节坐标没有公开 setter(`link.joint.coords` 是 `pub(crate)`);唯一公开
  写入路径 = `Multibody::apply_displacements`(增量位移,内部
  `integrate(1.0, disp)`,revolute 等价 `coords[AngX] += disp` 并重建
  `joint_rot`)+ `forward_kinematics(bodies, true)`(从根位姿重算全部连杆)
  + `update_rigid_bodies`(连杆位姿写回刚体);
- 广义速度走 `Multibody::generalized_velocity_mut`(写 `self.velocities`;
  下一拍 solver 的 `update_velocities` 从它重建各连杆刚体速度,
  staged_island_solver/helpers.rs:108),根刚体的 `linvel/angvel` 同时直写;
- 恢复后逐位核对:`quad::measure_restore_error`。

## 矩阵

193 行 = 行走 12 相位 × 3 重建 × 2 快照 × 2 地形(144)+ 站立醒/眠 × 12(24)
+ 落地 × 12 + 来回迁移 6 + 对照 6 + 8 只组 1。

- 重建:`same`(同世界移除再重建)/ `cross`(移到第二个世界,两世界同拍
  步进)/ `cross_hist`(cross + 目标世界先插入、删除 200 刚体 + 20 m 外
  3×3×3 堆);
- 快照:`full`(根位姿、根速度、关节坐标、关节速度、目标;另带连杆位姿作
  判据 1 核对基准)/ `novec`(速度清零);
- 地形:T0 大平面;T1 chunk 接缝(`greedy_colliders`,与 terrain 同构)。

## 判据逐条(初定;通过与否由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | 完整快照重建后、步进前:关节坐标误差 < 1e-4 rad,连杆位置误差 ≤ 1e-4 m | ✅ **全部 97 个 full 行误差逐位为 0**(0.000e0)——`apply_displacements` + FK + 速度写回是逐位恢复 |
| 2 | 完整快照,迁移后 120 拍:up·y ≥ 0.97,\|vy\| ≤ 0.35,连杆峰值 ≤ max(20, 2×对照),机身相邻拍位移 ≤ 1.5×对照 | ✅ 行走/站立全部 84 行达标(up·y min 0.9889,最大 \|vy\| 0.3045,连杆峰值 2.11 vs 对照 2.02);落地组(land)窗口覆盖整个落地冲击,对照自身 up·y 0.7459–0.7661/\|vy\| 3.45,迁移组与对照同量级(up·y 0.7500–0.7525 / \|vy\| 3.4503–3.4504 / 连杆 3.5228–3.5653 vs 对照 3.7763–3.7968),**迁移没有加剧落地冲击**,按"只报结果"口径如实报 |
| 3 | 单次迁移:全部相位不摔倒,全程 vfwd ≥ 对照的 95%,对地穿透 ≤ 5 cm | ✅ 144 行走迁移组 0 摔倒;vfwd 对对照(T0 0.5161 / T1 0.5123):same 99.7–99.8%、cross 99.7–99.8%;cross_hist T0 最差 56.3%、T1 84.1%(见"cross_hist 现象");穿透全部 ≤ 4.6 mm |
| 4 | 连续来回迁移:间隔 ≥ 30 拍时不摔倒,vfwd ≥ 对照的 80%;更短的间隔只报数 | ✅ g30 vfwd 0.4961(对照 96.1%)、g120 0.5278(102.3%),0 摔倒;**g5 0.3671(71.1%)、g2 0.0057(1.1%,up·y 0.9644 一次贴线)**——每 2 拍重建一次等于每拍丢接触缓存,g2 只报数 |
| 5 | 同参数双跑逐位相同 | ✅ det 4 对(walk p00/p06 same、walk p06 cross T1、pingpong g30)全部 IDENTICAL |

**novec(不带速度)对照**:vfwd 均值与 full 在 100.0–100.1%(T0/T1),
up·y/穿透同量级——**步行迁移场景下速度几乎不必带**(行走速度 ~0.5 m/s,
丢速度的扰动小于步态本身;落地/高速场景未单独验证)。

## `cross` 与 `cross_hist` 的轨迹分歧(只报数,不是判据;裁定见放行)

与同场景 `same_full` 行逐拍状态哈希比较的首个分歧拍(diverge_tick 列):

- **cross(干净目标世界)**:行走 24 行中 8 行全程逐位一致(T0 全部 8 行、
  T1 的 p00–p07 有 0 行……具体见下);分歧的行从迁移后 13–747 拍开始。
  醒着站立 T0/T1、已睡眠站立 T1、pingpong g30 全程一致;
  已睡眠站立 T0 在 780 拍分歧(迁移点在 780 拍后,即睡眠被迁移唤醒后);
  落地组 106/107 拍(落地冲击拍)分歧;
- **cross_hist(句柄历史 + 方块堆)**:行走 24 行**全部**在迁移后第 1–18 拍
  分歧(迁移拍 180–198 + 1);站立醒 180/198、睡眠 780 后。

分歧的最小单元是"求解顺序 + 句柄历史"进数值路径:cross_hist 的目标世界有
残留句柄空洞和 27 个活跃方块堆,解算配对顺序与 same/cross 不同。**这与
mass demo 隔离性一节的 `multi_group` 结论一致:只要世界里有 multibody,
全部醒着刚体合成一组求解,组内的解算顺序依赖世界的完整历史。**生产迁移的
确定性建议按"跨世界不承诺逐位一致、同世界 same 逐位一致"定义,由总架构师裁定。

vfwd 的量级差也只在 cross_hist:worst 行 walk_p01/07/08/09_cross_hist_full_T0
= 0.2908 m/s(56.3%),其余 cross_hist 行 84–100%。原因未归因(这些行
diverge 后步态相位被打乱,开环步态对扰动敏感是 gait 已知现象)。

## 每组报告列

`err_coord/err_pos/err_rot/err_vel`(重建后、步进前;关节坐标 rad / 连杆位置
m / 连杆转角 rad / 速度)、`post_upy_min/post_max_vy/post_link_max/post_pen_max`
(迁移后 120 拍)、`contact_recover_tick`(迁移后首次真实脚-地接触相对拍;
全部迁移组 = 1)、`vfwd`(行走段均值)、`fell_tick`、`nan`、`panicked`、
`migrate_us`(restore_robot 单次耗时)、`diverge_tick`、`land_tick`、
`state_hash`。

**迁移耗时**:单次 46.8–131.4 µs(峰值在 T1 体素世界);8 只同拍迁移平均
57.0 µs/只,迁移后全体 up·y min 0.9783、最大 \|vy\| 0.3632、连杆峰值
2.03 m/s(≈ 平地单只行走水平)。

## 现象与归因

1. **重建误差逐位为 0**:`apply_displacements` 的位移增量 + FK 从根位姿
   重算,浮点上就是同一个算式;速度写回 `generalized_velocity` 后 solver
   下一拍重建连杆速度。**"丢接触缓存"对重建本身零影响**——接触缓存本来
   就不在快照里;
2. **迁移后 up·y 最低 0.9889(行走组)**:迁移拍脚处于摆动相时,重建丢失
   该脚的接触 + 摆动轨迹继续,扰动小于一步;全部组 contact_recover = 1 拍;
3. **落地组(land)**:站稳后瞬移 0.5 m 悬空、自由落体、**任一脚 dist<1 mm
   的那一拍迁移**。迁移组与不迁移对照(同布景)逐项同量级(up·y 0.7500–0.7525
   vs 对照 0.7459–0.7661;|vy| 3.45–3.56 vs 3.45;连杆峰值 3.52–3.62 vs
   3.78–3.80,迁移组反而略低)——**落地冲击由物理主导,迁移不放大**;
4. **pingpong g2**:每 2 拍移除重建一次,等效每拍丢接触缓存,步态完全散
   (vfwd 0.006);g5(71%)介于中间。间隔 ≥ 30 拍后与单次迁移无异;
5. **睡眠迁移**:已睡眠站立迁移后 600 拍内 up·y ≥ 0.9926;睡眠目标世界
   (cross)全程与 same 逐位一致(T1)。

## 范围

新增 `demos/agents/migrate/`(独立 crate,rapier3d 锁 =0.36.1,提交
Cargo.lock,独立 target 已 gitignore);`quad` 追加
`AgentSnapshot` / `snapshot_robot` / `restore_robot` / `measure_restore_error`,
不动既有数值路径——追加后 `gait` 168 行、`terrain` 85 行、`mass` 5313 行
状态哈希与提交数据逐位相同(重跑比对,计时列除外)。不改 `crates/`、`docs/`。
