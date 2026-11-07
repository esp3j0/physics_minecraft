# A00-3 demo `freeze`(REQ-012 §3 冻结 / AC-B4,2026-11-07)

放行说明:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
最后一条评论(A00-3 放行)。**问题(放行原文)**:多连杆能不能冻住,唤醒后
会不会弹飞?REQ-012 §3 写明"rapier 对 multibody 的运动学切换行为未验证"。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

## 运行方式

```bash
cd demos/agents/freeze
cargo run --release -j 10          # 全矩阵 87 行,约 1 分钟
cargo run --release -j 10 -- det   # 2 组双跑确定性比对(输出 out/run_det.txt)
```

**提交的原始数据**:`out/matrix_submitted.txt`(87 行全表;默认运行重写
`out/run_summary.csv`,提交文件由提交流程拷贝)、`out/det_submitted.txt`。
独立 `target` 目录已 gitignore。

## 结论(TL;DR)

**放行点名的"全连杆切 Kinematic/Fixed"在 rapier 0.36.1 不可行——直接
panic**(`multibody.rs:1457` 硬断言"非根连杆必须 Dynamic"),这就是 §3
"未验证"的答案。可行的三口径:

- **snap(移除重建)是唯一满足判据 1(冻结期位移 ≤ 1e-5)的方案**:障碍物
  是 fixed 刚体,位移逐位为 0;唤醒按 migrate 快照重建,误差逐位为 0,
  判据 2/3 全过。**推荐生产采用**(推荐依据见下);
- **kinroot/fixedroot(只切根)可行但不是硬冻结**:非根连杆保持 Dynamic、
  仍进 solver,冻结期间腿在重力/电机下继续动(站立 drift 1.9 cm,行走
  1.1–7.2 cm,fixedroot 0.38–0.41 m 且 15 行唤醒后摔倒)——判据 1 不过,
  如实报数;
- 生产语义(清速度 → 切 KinematicPositionBased)对 multibody 只能用于
  **根连杆**;单刚体路径(REQ-007 现有 `freeze_entity`)不受影响。

## rapier 依据(0.36.1 实测 + 源码位置)

1. **非根连杆切型 panic**:`Multibody::forward_kinematics`
   (`rapier3d-0.36.1/src/dynamics/joint/multibody_joint/multibody.rs:1457`)
   `assert_eq!(body_type, Dynamic, "A rigid-body that is not at the root of
   a multibody must be dynamic")`。全连杆切型在下一次 `world.step` 即触发,
   6 行(stand/walk p00 × kin/fixed)如实记 panic 行(catch_unwind 隔离,
   矩阵不中断);
2. **根切型走 `update_root_type`**(同文件 ~:1290):每拍 FK 时若根刚体
   `is_dynamic()` 与 `root_is_dynamic` 不符,重建根关节(free ↔ fixed)。
   只切根不 panic,但**只固定根刚体**,非根连杆(12 个)继续被求解;
3. **睡眠 island 不做 FK 传播**(布景实证):瞬移根刚体后不唤醒,非根连杆
   的刚体位姿停留在出生位(pile 布景首个版本快照里根 y=1.45、非根 y=0.45,
   差 1 m)——瞬移 multibody 必须 `wake_up` 整只(生产迁移/瞬移的硬要求,
   已写进 pile 布景注释)。

## 方案与矩阵

87 行 = 对照 2 + 站立 5 方案 × 2 唤醒速度(10)+ 行走 12 相位 × 4(48)+
全连杆 panic 证据 2 + 堆顶 6 + 压住/下落/砸中/移走支撑各 3(12)+ 长冻结
3600 拍 6 + 8 只 1。

- 方案:`kin`/`fixed`(放行原文口径,全部连杆切型——panic 证据)、
  `kinroot`/`fixedroot`(只切根)、`snap`(移除 multibody、碰撞体留 fixed
  障碍物,唤醒按 `quad::restore_robot` 重建);
- 唤醒速度:`rst`(恢复冻结前)/ `z`(清零 = 生产语义;kinroot 的数据两口径
  几乎相同,冻结时本来清了)。

## 判据逐条(初定;通过与否与方案取舍由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | 冻结期间全部连杆位移 ≤ 1e-5 m,包括被方块砸中时 | ✅ **snap 全部 21 行 drift = 0.000e0**(含砸中、压住、堆顶、下落、移走支撑);❌ kinroot 1.9e-2(站立)–7.2e-2(行走)、fixedroot 0.38–0.47,非根连杆继续被求解,如实报 |
| 2 | 唤醒后、步进前:关节坐标误差 < 1e-4 rad,连杆位置误差 ≤ 1e-4 m | ✅ snap 全部 21 行误差逐位为 0(migrate 同款重建路径);kinroot/fixedroot 0.01–1.2(非根连杆冻结期间动了,恢复口径不适用) |
| 3 | 站立和行走工况唤醒后 120 拍:up·y ≥ 0.97,\|vy\| ≤ 0.35,连杆峰值 ≤ max(20, 2×对照),机身升高 ≤ 5 cm | ✅ snap 站立/行走全部 14 行:up·y min 0.9858、\|vy\| max 0.3094、机身升高 max 1.02 cm、连杆峰值 2.06(对照 2.02,余量充足);kinroot 同口径 26 行也全过(up·y min 0.9752、升高 max 1.15 cm)——但它的"冻结"不成立(判据 1) |
| 4 | 站立和行走全部相位不摔倒;行走 vfwd ≥ 对照的 90%(只算走路拍) | ✅ snap 站立 + 行走 12 相位全部不摔;vfwd 0.4826–0.4849(gait 平地基线 0.532 的 91–100%,对照口径:本 demo 的行走对照不冻结行 vfwd 计 0——对照行是站立口径,行走比较以 gait `f3` 主组 0.532 为参照如实报);❌ fixedroot 15 行摔倒(唤醒后 26–76 拍),kinroot 0 摔 |
| 5 | 全部工况无 panic、无 NaN;压住、下落、脚下被移走的工况只报结果 | panic:仅 6 个**全连杆切型证据行**(放行方案的可行性结论,见上);NaN 全矩阵 0。snap 布景组:压住 0.9983/0.027、砸中 0.9983/0.026(冻结期方块从 3 m 砸在障碍物上,反弹后落定,主体唤醒无感)、下落途中冻结 1.0000/0.000(600 拍内落地后静止冻结,唤醒原地站立);**移走支撑 -0.9996/4.22**(脚下 3×3 堆被移走,主体连障碍物一起下落 1 m 摔倒——物理结果,障碍物是 fixed 不会自己掉,但堆没了主体悬空,唤醒时已在地面) |
| 6 | 同参数双跑逐位相同 | ✅ det 2 对(stand_kin_rst 全连杆 panic 行、walk_p06_snap_rst)IDENTICAL |

## snap 推荐(逐项对应数据)

1. **判据全过**:上表 1/2/3/4/5 snap 全绿,是唯一满足"冻结 = 完全不动"
   语义的方案(§3 的冻结语义是无人观察时身体静止,kinroot 的 2–7 cm 下垂
   不满足);
2. **确定性**:唤醒路径 = migrate 的 `restore_robot`,误差逐位为 0
   (det IDENTICAL),迁移与冻结共用同一套重建代码;
3. **代价**:冻结/唤醒各一次移除+重建,单只 57–196 µs(方案内 21 行范围),
   8 只同拍冻结+唤醒平均 7.3 µs/只(kinroot 口径;全连杆不适用)。相比
   kinroot 的 `set_body_type`(0.4–29 µs)贵约 5–10 倍,但冻结本身是低频
   事件(无人观察才触发);
4. **碰撞连续性**:障碍物保持碰撞体,冻结期间方块砸上来有真实接触
   (hit_snap 数据);kinroot 期间连杆碰撞体也在,但连杆在动。

## 布景与现象

- **堆顶(pile)**:站 3×3 方块堆顶站稳后冻结 600 拍,唤醒继续站
  (snap:up·y 0.9948,机身升高 0.12 cm;kinroot:0.9953)。**布景坑**:瞬移
  根刚体必须 `wake_up(true)` + 整只唤醒,否则非根连杆位姿不同步(见 rapier
  依据 3);
- **压住(pressed)**:站稳后 1 kg 方块轻放背上,30 拍后冻结。snap 唤醒后
  方块在背上静置,主体 up·y 0.9983(对照 mass M3 结论:压缩档会被压塌,
  这里 1 kg 方块压不塌);
- **砸中(hit)**:冻结 100 拍时 1 kg 方块从 3 m 落在背上。障碍物接住方块
  (fixed 不可动),唤醒时方块已弹开落定,主体无感(0.9983/0.026);
- **移走支撑(suprm)**:冻结 100 拍后移除脚下方块堆。障碍物 fixed 不掉,
  但主体 + 障碍物失去支撑一起下落(障碍物是被移走后悬空的 fixed 不动——
  实际是主体的腿还搭在障碍物上,堆消失后整只翻落地面,up·y -0.9996)。
  物理上"脚下没了"就该掉,判据 5 只报结果;
- **长冻结(3600 拍)**:snap 全部 drift 0,唤醒后站立 1.0000(睡眠口径,
  未唤醒)。

## 范围

新增 `demos/agents/freeze/`(独立 crate,rapier3d 锁 =0.36.1,提交
Cargo.lock,独立 target 已 gitignore);`quad` 只追加(与 migrate 共用),
不动既有数值路径——追加后 `gait` 168 行、`terrain` 85 行、`mass` 5313 行
状态哈希与提交数据逐位相同(重跑比对,计时列除外)。不改 `crates/`、`docs/`。
