# A00-3 demo `freeze`(REQ-012 §3 冻结 / AC-B4,2026-11-07;R1 返修)

放行与验收:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
(A00-3 R1 放行;上一条验收里的 F1–F10 与"冻结方案裁定"是本轮要修的问题)。
**问题(放行原文)**:多连杆能不能冻住,唤醒后会不会弹飞?REQ-012 §3 写明
"rapier 对 multibody 的运动学切换行为未验证"。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

**本 README 的全部总结数字由 `scripts/summary.py` 从提交数据
`out/matrix_submitted.txt` / `out/det_submitted.txt` 计算**。

## 运行方式

```bash
cd demos/agents/freeze
cargo run --release -j 10          # 全矩阵 88 行(写 out/run_summary.csv)
cargo run --release -j 10 -- det   # 全矩阵双跑,逐行比较状态哈希(out/run_det.txt)
python scripts/summary.py          # 从提交数据算 README 数字
```

**提交的原始数据**:`out/matrix_submitted.txt`(88 行全表)、
`out/det_submitted.txt`(88 对)。`out/*` gitignore,只有这两个白名单。
独立 `target` 已 gitignore。

## R1 修正的 F1–F10(逐条)

| # | 验收问题 | 修正 |
|---|---|---|
| F1 | 障碍物形状按 `i % 4` 错配、每连杆一个刚体 | 障碍物 = **一个** fixed 刚体;全部连杆的**真实碰撞体**(形状 `shared_shape().clone()`、局部偏移 `position_wrt_parent`、摩擦、恢复系数)按 collider 的 parent 逐个搬到 compound(links[0] = 机身,colliders 17 个与 links 13 个**不对齐**——小腿带 2 个 collider,旧代码错在这里);`obstacle_shape_ok` 列用代码逐个核对数量、形状特征、世界位姿 |
| F2 | falling 没唤醒(snap 还不步进世界) | falling 在半空冻结(断言冻结时最低脚离地 > 0.05 m),冻结期间世界照常步进,唤醒才落地;`woke_tick` 列,没唤醒的行 panic 不进矩阵 |
| F3 | `*_long_*` 没唤醒(总拍数不够) | 长冻结组总拍数 = 冻结拍 + 3600 + 600(唤醒后至少 600 拍);`stand_long_snap/walk_p06_long_snap` 实测 woke_tick = 3600 |
| F4 | 行走对照没走(哈希与站立对照相同) | `control_walk_pXX` 12 相位各一条,真行走;vfwd 与冻结组同起点同拍数(对照窗口 = 名义唤醒拍起);站立对照强制醒着 |
| F5 | 目标只在偶数拍写,奇数拍唤醒失力一拍 | 目标每拍都写(冻结期间 snap 不在/kinroot 不求解,照写保持口径) |
| F6 | `z` 只清关节速度、snap 没跑 `z` | `WakeVel::Zero` 把根速度 + 关节速度都清零(`snapshot(wake_vel)` 口径);snap 12 相位 rst、z 都跑 |
| F7 | 8 只组只测 kinroot、耗时刻在 snap 名下 | `eight_snap_rst` + `eight_kinroot_rst` 各一组,`freeze_us`/`wake_us` 分开;8 只不冻结对照 `eight_control` |
| F8 | pressed/hit 布景与说明不一致 | pressed = 放方块 30 拍后冻结(walk0−30 放,walk0+30 冻);hit = 冻结后 100 拍生成方块(`scene_note` 列记录实际事件拍);两组报唤醒后方块位置/睡着/是否在背上 |
| F9 | "1 kg 方块压不塌"没有依据 | 删掉;pressed 现在真的压 30 拍再冻结(机身 0.2777 m 起步),唤醒后方块仍在背上(block_on_back=true),主体 up·y 0.9551——压塌与否只报数 |
| F10 | suprm 说明自相矛盾 | 障碍物 fixed 不掉;支撑堆被移走后,主体唤醒时已摔到地面(end_body_y 0.0569、up·y −0.9996),只报结果 |

## rapier 依据(0.36.1 实测,方案可行性)

1. **非根连杆切型 panic**(放行点名的 kin/fixed 口径):`Multibody::
   forward_kinematics`(`rapier3d-0.36.1/src/dynamics/joint/multibody_joint/
   multibody.rs:1457`)硬断言"A rigid-body that is not at the root of a
   multibody must be dynamic",下一次 `world.step` 即触发。4 个证据行
   (`stand_kin_rst`、`walk_p00_kin_rst`、`stand_fixed_rst`、
   `walk_p00_fixed_rst`)如实记 panic 行(catch_unwind 隔离);det 双跑
   IDENTICAL(两次同 panic,不算确定性证据——R1 口径);
2. **只切根不 panic**(`update_root_type`,同文件 ~:1290,FK 时按根类型重建
   根关节),但**只固定根刚体**,12 个非根连杆继续被求解——**不是冻结**
   (R1 裁定否决,保留站立 + 12 相位 rst 作记录);
3. **睡眠 island 不做 FK 传播**(布景坑,生产瞬移必须整只 `wake_up(true)`):
   瞬移根刚体后不唤醒,非根连杆的刚体位姿停留在出生位(pile 布景首版快照
   根 y=1.45、非根 y=0.45,差 1 m)。

## 方案与矩阵(88 行)

对照 13(站立 1 + 行走 12)+ kin/fixed panic 证据 4 + kinroot/fixedroot 记录
26(站立 2 + 12 相位 × 2)+ snap 站立/行走 26(站立 rst/z + 12 相位 × rst/z)
+ 长冻结 4 + 堆顶 4 + 压住/下落/砸中/移走支撑 8 + 8 只组 3(scripts/
summary.py 的行数计数:snap 35、kinroot 22、fixedroot 13、control 13、
eight 3)。

## 判据逐条(R1 口径;通过与否与方案取舍由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | `snap` 冻结期间全部障碍物位移 ≤ 1e-5 m(含被砸中),障碍物与连杆碰撞体逐个相符 | ✅ 35 个 snap 行 drift 全部 0.000e0(含砸中/压住/堆顶/下落/移走支撑;summary.py 判据1);`obstacle_shape_ok=true` 35 行、0 行非 true(数量/形状特征/世界位姿逐个相符) |
| 2 | 唤醒后、步进前:关节坐标 < 1e-4 rad、连杆位置 ≤ 1e-4 m | ✅ snap 全部行误差逐位为 0(0.000e0;summary.py 判据2;migrate 同款 `restore_robot` 重建路径) |
| 3 | 站立和行走工况唤醒后 120 拍真实窗口:up·y ≥ 0.97、\|vy\| ≤ 0.35、连杆 ≤ max(20, 2×对照)、机身升高 ≤ 5 cm | ✅ 站立(n=2):up·y min 0.9977、\|vy\| max 0.0278、连杆 0.1444、升高 max 0.0007 m;行走(n=25):up·y min 0.9810、\|vy\| max 0.2444、连杆 2.0310(对照 2.0135)、升高 max 0.0097 m(summary.py 判据3) |
| 4 | 站立和行走全程不摔;行走 vfwd ≥ 同相位、同行走拍数对照的 90% | ✅ snap 摔倒行 = 无;逐相位 vfwd 对同相位对照 **97.0%–99.9%**(对照均值 0.5417、snap 行走均值 0.5347;summary.py 判据4)。❌ 否决记录:fixedroot 12 行摔倒(kinroot 0 行) |
| 5 | 全部 `snap` 工况无 panic、无 NaN;压住、下落、砸中、脚下被移走只报结果(必须真走到唤醒) | ✅ NaN(panic 证据行除外)= 无;4 个 kin/fixed panic 行是放行方案的可行性结论本身;压住/下落/砸中/移走支撑全部真唤醒(woke_tick 600/600/671/600,summary.py 场景) |
| 6 | 全矩阵双跑逐行相同 | ✅ det 88 对全部 IDENTICAL(summary.py det;panic 行两跑同 panic,按 R1 口径不算确定性证据,但哈希一致已记录) |

## 否决记录:kinroot / fixedroot(不是冻结)

- drift:kinroot max **0.3046 m**、fixedroot max **0.4078 m**(summary.py 否决
  记录)——非根连杆冻结期间在重力/电机下继续动,判据 1 不过;
- fixedroot 12 行唤醒后摔倒(26–76 拍内;summary.py 否决记录),kinroot 0 摔;
- 两方案按 R1-8 只保留站立 + 12 相位 rst,其余工况未跑。

## 场景报数(snap;summary.py 场景)

| 场景 | woke_tick | 唤醒后 up·y / \|vy\| | 终态 |
|---|---|---|---|
| pressed(压住:方块轻放 30 拍后冻结) | 600 | 0.9551 / 0.1117 | 方块仍在背上(on_back=true、block_asleep=false),机身 rise 0.0012 m;放方块时机 body_y=0.2777(冻结前已承重 30 拍) |
| hit(冻结 100 拍后 1 kg 方块从 3 m 落背上) | 600 | 0.9983 / 0.0264 | 方块砸在障碍物上,唤醒时已落定睡着(y=0.4999、asleep=true、on_back=true),主体无感 |
| falling(半空冻结,唤醒才落地) | 671 | 0.7538 / 3.5313 | 冻结时脚离地(断言过),600 拍后唤醒、落地冲击 \|vy\| 3.53 ≈ 0.5 m 自由落体末速,窗口 up·y 0.7538 是落地冲击本身,只报结果 |
| suprm(冻结 100 拍后移走脚下方块堆) | 600 | −0.9996 / 4.2231 | 障碍物 fixed 不掉,但支撑堆没了,主体摔到地面(end_body_y 0.0569),只报结果 |
| pile(站 3×3 堆顶) | 600 | 0.9955 / 0.0319 | 唤醒继续站,rise 0.0021 m |
| 长冻结 3600 拍(stand / walk p06) | 3600 | 0.9983 / 0.1918 | drift 0、唤醒后正常站立/行走,与 600 拍无差 |

## 8 只组与耗时

- `eight_snap_rst`:freeze 34.93 µs/只、wake 35.48 µs/只,唤醒后全体
  up·y 0.9982,**状态哈希与 8 只不冻结对照 `eight_control` 逐位相同**
  (5561f81e12ae0505)——snap 冻结/唤醒一整轮后群组轨迹与从未冻结一致;
- `eight_kinroot_rst`:freeze 0.44 µs/只、wake 5.20 µs/只(set_body_type
  便宜得多,但 drift 0.30 m,不是冻结);
- 单只 snap:freeze 34.9–104.9 µs、wake 35.5–71.3 µs(summary.py 耗时)。

## 方案推荐(逐项对应数据)

**snap**(移除 multibody + 全部真实碰撞体留一个 fixed 障碍物,唤醒按
migrate 快照重建):

1. 判据 1"冻结 = 完全不动"语义只有 snap 满足(drift 逐位 0,kinroot
   0.18–0.30 m 不满足);
2. 唤醒路径与迁移共用 `restore_robot`,误差逐位 0、8 只组哈希与不冻结对照
   逐位相同;
3. 代价:单只冻结+唤醒 ≈ 70–175 µs,8 只同拍 ≈ 35 µs/只,比 kinroot 的
   set_body_type 贵 ~10 倍——冻结是无人观察才触发的低频事件;
4. 冻结期间障碍物保持真实碰撞(hit 场景有真实接触,方块被接住落定)。

`kinroot` 可作"轻量待机"口径(判定不要求不动时);`fixedroot` 不建议
(12 行摔);`kin`/`fixed` 在 rapier 0.36.1 不可行(panic,§3"未验证"的答案)。

## 范围

只动 `demos/agents/freeze/`(代码 + 提交数据 + scripts/)与 `quad` 共用
(migrate 轮已追加,`restore_robot` 写回 targets 本轮同样生效);`quad` 追加后
`gait` 168 行、`terrain` 85 行、`mass` 5313 行状态哈希与提交数据逐位相同
(全量重跑比对,计时列除外)。不改 `crates/`、`docs/`。
