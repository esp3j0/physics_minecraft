# A00-3 demo `freeze`(REQ-012 §3 冻结 / AC-B4,2026-11-07;R2 返修)

放行与验收:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
(A00-3 R2 放行;上一条验收里的 FR1–FR9 是本轮要修的问题)。
**问题(放行原文)**:多连杆能不能冻住,唤醒后会不会弹飞?REQ-012 §3 写明
"rapier 对 multibody 的运动学切换行为未验证"。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

**本 README 的全部总结数字由 `scripts/summary.py` 从提交数据
`out/matrix_submitted.txt` / `out/det_submitted.txt` /
`out/eight_per_robot_submitted.txt` 计算**(脚本按完整名字段筛行、断言每行
列数 == 列头数(FR8)、断言对照行 `eight_control` 的 freeze_us/wake_us 为
"-"(FR1);各数字来自脚本的对应 `==` 小节)。

## 运行方式

```bash
cd demos/agents/freeze
cargo run --release -j 10          # 全矩阵 88 行(写 out/run_summary.csv)
cargo run --release -j 10 -- det   # 全矩阵双跑,逐行比较状态哈希(out/run_det.txt)
python scripts/summary.py          # 从提交数据算 README 数字
```

**提交的原始数据**(默认运行写 `out/run_*`,不覆盖;提交件单独拷贝,
三个白名单):`out/matrix_submitted.txt`(88 行全表)、
`out/det_submitted.txt`(88 对)、`out/eight_per_robot_submitted.txt`
(8 只组逐只行,24 行,与主表同列头)。

## R2 修正的 FR1–FR9(逐条)

| # | 验收问题 | 修正 |
|---|---|---|
| FR1 | `eight_control` 也冻结了(`run_eight` 不看对照标志) | 对照行不进任何冻结/唤醒分支;`freeze_us`/`wake_us` 列写 `-`(summary.py 断言);对照 upy=1.0000(站立未受扰) |
| FR2 | 8 只组按下标取碰撞体(17 collider 对 13 link 错配) | 8 只组与单只**同一个**冻结函数 `freeze_snap_handles`(按 collider 的 parent 刚体匹配;句柄组快照入参);`freeze_one_snap` 下标版已删 |
| FR3 | 8 只组列写死(drift=0、shape=true、woke=freeze_len、误差/哈希只算第 0 只) | 全列逐只真测量:逐只 drift(kinroot 读连杆位移)、逐只 shape 核对(冻结后立刻,与单只同函数同口径)、逐只唤醒误差、逐只窗口指标、哈希含 8 只全部位姿+12 关节角;逐只行提交为 `out/eight_per_robot_submitted.txt`(白名单已加) |
| FR4 | `hit` 的方块在冻结拍就生成,`scene_note` 写的拍号与代码不符 | 方块在**冻结后第 100 拍**生成(`hit_block_drop_tick=280`,从代码实际事件拍取值);`pressed` 放方块后第 30 拍冻结(`pressed_block_tick=150` = walk0−30) |
| FR5 | "在背上"用高度差判定(方块 y=0.4999 是落在地面) | 改 `world.contact_pairs()` narrow_phase 接触对(方块 collider × 主体 collider,manifold dist<5mm);`pressed`/`hit` 终态 on_back=true 是真实接触对 |
| FR6 | `pressed` 只报终态;报数时机与说明不符 | 唤醒拍、+120、+600 各报一次方块位置/睡着/on_back/机身高度(scene_note 里 `t+0/t+120/t+600` 三段;总拍数 +1 让 t+600 在循环内) |
| FR7 | `suprm` 没报唤醒时离地高度,README 写"唤醒时已摔到地面" | 新增三项:唤醒拍机身离地高度 1.2713 m、唤醒后首次触地 +27 拍、触地时 up·y 0.9549;README 改为"唤醒后从 1.27 m 摔下,+27 拍触地" |
| FR8 | CSV 列头 22 列、数据 23 列(`active_order_end` 落进 `state_hash` 列) | 删掉 `wake_us` 后多余的 `-` 占位;cells 数 = header 数;summary.py 断言全表 88 行列数一致 |
| FR9 | compound 只有一份摩擦/恢复系数;错误 `debug_assert`(13==13)与"同长 13"注释 | 障碍物 = 一个 fixed 刚体**逐个挂**原真实碰撞体(`insert_collider(_, Some(body))`,每个 collider 自带形状、相对位姿、摩擦、恢复系数);`obstacle_shape_ok` 逐 collider 核对含**材质**(fric/rest 与原 collider 一致);错误 assert 与注释已删 |

**行走对照窗口口径**(R1 遗留,写明):对照行 vfwd 从名义唤醒拍起统计
(对照此时已走了 600 拍以上),冻结组唤醒后只走过 600 拍;两者都接近稳态,
逐相位比 97.0%–99.9% 的结论按此口径。

## rapier 依据(0.36.1 实测,方案可行性)

1. **非根连杆切型 panic**:`Multibody::forward_kinematics`(`rapier3d-0.36.1/
   src/dynamics/joint/multibody_joint/multibody.rs:1457`)硬断言"A rigid-body
   that is not at the root of a multibody must be dynamic",下一次
   `world.step` 即触发。4 个证据行(`stand_kin_rst`、`walk_p00_kin_rst`、
   `stand_fixed_rst`、`walk_p00_fixed_rst`)如实记 panic 行(catch_unwind
   隔离);det 双跑 IDENTICAL(两次同 panic,不算确定性证据——R1 口径);
2. **只切根不 panic**(`update_root_type`),但只固定根刚体,12 个非根连杆
   继续被求解——**不是冻结**(R1 裁定否决,保留站立 + 12 相位 rst 作记录);
3. **睡眠 island 不做 FK 传播**(布景坑,生产瞬移必须整只 `wake_up(true)`):
   瞬移根刚体后不唤醒,非根连杆的刚体位姿停留在出生位;
4. **一个刚体挂多个独立碰撞体**:`PhysicsWorld::insert_collider(collider,
   Some(body))`(physics_world.rs:310)逐 collider 各带材质——FR9 的写法,
   生产脚摩擦与机身不同时用这条路径。

## 方案与矩阵(88 行)

对照 13(站立 1 + 行走 12)+ kin/fixed panic 证据 4 + kinroot/fixedroot 记录
26(站立 2 + 12 相位 × 2)+ snap 站立/行走 26(站立 rst/z + 12 相位 × rst/z)
+ 长冻结 4 + 堆顶 4 + 压住/下落/砸中/移走支撑 8 + 8 只组 3(scripts/
summary.py 的行数计数:scheme=snap 33、kinroot 22、fixedroot 13、control 13、
eight 3;按完整名字段拆分统计)。

## 判据逐条(R2 口径;通过与否与方案取舍由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | `snap` 冻结期间全部障碍物位移 ≤ 1e-5 m(含被砸中),障碍物与连杆碰撞体逐个相符(含材质) | ✅ 33 个 snap 行 drift 全部 0.000e0(含砸中/压住/堆顶/下落/移走支撑;summary.py 判据1);`obstacle_shape_ok=true` 33/33(数量/形状特征/世界位姿/**逐 collider 材质**逐个相符) |
| 2 | 唤醒后、步进前:关节坐标 < 1e-4 rad、连杆位置 ≤ 1e-4 m | ✅ snap 全部行误差逐位为 0(0.000e0;summary.py 判据2;migrate 同款 `restore_robot` 重建路径) |
| 3 | 站立和行走工况唤醒后 120 拍真实窗口:up·y ≥ 0.97、\|vy\| ≤ 0.35、连杆 ≤ max(20, 2×对照)、机身升高 ≤ 5 cm | ✅ 站立(rst,n=2):up·y min 0.9977、\|vy\| max 0.0278、连杆 0.1444、升高 max 0.0007 m;行走(n=24):up·y min 0.9810、\|vy\| max 0.2444、连杆 2.0310(对照 2.0135)、升高 max 0.0097 m(summary.py 判据3) |
| 4 | 站立和行走全程不摔;行走 vfwd ≥ 同相位、同行走拍数对照的 90% | ✅ snap 摔倒行 = 无;逐相位 vfwd 对同相位对照 **97.0%–99.9%**(对照均值 0.5417、snap 行走均值 0.5345;summary.py 判据4)。❌ 否决记录:fixedroot 12 行摔倒(kinroot 0 行) |
| 5 | 全部 `snap` 工况无 panic、无 NaN;压住、下落、砸中、脚下被移走只报结果(必须真走到唤醒) | ✅ NaN(panic 证据行除外)= 无(summary.py 判据5);4 个 kin/fixed panic 行是放行方案的可行性结论本身;压住/下落/砸中/移走支撑全部真唤醒(woke_tick 600/600/671/600,summary.py 场景) |
| 6 | 全矩阵双跑逐行相同 | ✅ det 88 对全部 IDENTICAL(summary.py det;panic 行两跑同 panic,按 R1 口径不算确定性证据,但哈希一致已记录) |

## 否决记录:kinroot / fixedroot(不是冻结)

- drift:kinroot max **0.2674 m**、fixedroot max **0.4078 m**(summary.py
  否决记录;kinroot 逐只 drift 是 8 只组本轮新增的真测量,单只行走行
  0.3046→本表为站立+12 相位口径的 max)——非根连杆冻结期间在重力/电机下
  继续动,判据 1 不过;
- fixedroot 12 行唤醒后摔倒(26–76 拍内;summary.py 否决记录),kinroot 0 摔;
- 两方案按 R1-8 只保留站立 + 12 相位 rst,其余工况未跑。

## 场景报数(snap;summary.py 场景;scene_note 拍号 = 代码实际事件拍)

| 场景 | woke_tick | 唤醒后 up·y / \|vy\| | 终态 |
|---|---|---|---|
| pressed(放方块 walk0−30=150 拍,冻结 180 拍) | 600 | 0.9560 / 0.1200 | 三次报数 t+0/t+120/t+600:方块始终在背上(narrow_phase 接触对 on_back=true),t+600 后方块睡着(y=0.6590) |
| hit(冻结后第 100 拍 = tick 280 生成 1 kg 方块,3 m 落背) | 600 | 0.9878 / 1.2118 | t+0 方块砸中障碍物(y=0.8322 弹起中)、t+120 落到 y=0.6452、t+600 y=0.6269 仍与障碍物接触(on_back=true);主体 up·y 0.9878。**勘误(#25 总架构师)**:不是"无感"。机身高度从唤醒时的 0.2756 m 被压到 +120 拍 0.0603 m、+600 拍 0.1097 m,和 `pressed` 一样被压低 |
| falling(半空冻结,唤醒才落地) | 671 | 0.7538 / 3.5313 | 冻结时脚离地(断言过),600 拍后唤醒、落地冲击 \|vy\| 3.5313 ≈ 0.5 m 自由落体末速,窗口 up·y 0.7538 是落地冲击本身,只报结果 |
| suprm(冻结 100 拍后移走脚下方块堆) | 600 | −0.9996 / 4.2231 | **唤醒时主体还在 1.2713 m 半空**(FR7:冻结期间障碍物 fixed 不掉),唤醒后 +27 拍首次触地、触地时 up·y 0.9549,随后翻倒(up·y −0.9996 是翻倒终态,只报结果) |
| pile(站 3×3 堆顶) | 600 | 0.9956 / 0.0335 | 唤醒继续站,rise 0.0011 m |
| 长冻结 3600 拍(stand / walk p06) | 3600 | 0.9983 / 0.1918 | drift 0、唤醒后正常站立/行走,与 600 拍无差 |

## 8 只组与耗时

- `eight_snap_rst`:freeze 52.33 µs/只、wake 36.53 µs/只,唤醒后全体
  up·y 0.9982,逐只 drift 0、逐只 shape=true、逐只误差 0(逐只行 =
  `out/eight_per_robot_submitted.txt`,24 行 = 3 组 × 8 只);
- `eight_control`(FR1:**真不冻结**):freeze_us/wake_us = `-`
  (summary.py 断言);窗口统计同口径:up·y 0.9986、连杆 0.0817(真测量,
  非占位)。哈希 `fd23ac79…` 与 `eight_snap_rst`(`3cb63276…`)**不同**——
  R1 的"8 只组哈希与不冻结对照逐位相同"是在两行都冻结的 bug 下得出的,
  已作废;本轮对照真实不冻结,窗口指标与冻结组同量级(up·y 0.9982 vs
  0.9986);
- `eight_kinroot_rst`:freeze 0.45 µs/只、wake 5.18 µs/只,逐只 drift
  max 0.2674 m——不是冻结;
- 单只 snap:freeze 44.3–65.6 µs、wake 36.1–61.5 µs(summary.py 耗时)。

## 方案推荐(逐项对应数据)

**snap**(移除 multibody + 全部真实碰撞体逐个挂到一个 fixed 障碍物刚体,
各 collider 自带材质;唤醒按 migrate 快照重建):

1. 判据 1"冻结 = 完全不动"语义只有 snap 满足(drift 逐位 0,kinroot
   0.2674 m 不满足);
2. 唤醒路径与迁移共用 `restore_robot`,误差逐位 0;
3. 代价:单只冻结+唤醒 ≈ 80–127 µs,8 只同拍 ≈ 89 µs/只,比 kinroot 的
   set_body_type 贵 ~百倍——冻结是无人观察才触发的低频事件;
4. 冻结期间障碍物保持真实碰撞、**每个 collider 自带摩擦/恢复**(FR9 写法,
   hit 场景方块与障碍物有真实接触并停在背上)。

`kinroot` 可作"轻量待机"口径(判定不要求不动时);`fixedroot` 不建议
(12 行摔);`kin`/`fixed` 在 rapier 0.36.1 不可行(panic,§3"未验证"的答案)。

## 范围

只动 `demos/agents/freeze/`(代码 + 提交数据 + scripts/ + .gitignore 白名单);
`quad` 本轮未改(R2 放行)。不改 `crates/`、`docs/`。
