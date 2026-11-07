# A00-3 demo `migrate`(REQ-012 §3 迁移 / AC-B3,2026-11-07;R1 返修)

放行与验收:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
(A00-3 R1 放行;上一条验收里的 M1–M10 是本轮要修的问题)。**问题(放行
原文)**:丢掉接触缓存重建后,四足会不会跳变或摔倒?要验证的设计:迁移信封
只带身体哈希、根状态、关节坐标、关节速度、最后的目标,整个主体同一拍迁移。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

**本 README 的全部总结数字由 `scripts/summary.py` 从提交数据
`out/matrix_submitted.txt` / `out/det_submitted.txt` 计算**(数据要求:
"README 的每个总结数字都用脚本从提交文件算出来再写")。

## 运行方式

```bash
cd demos/agents/migrate
cargo run --release -j 10          # 全矩阵 420 行(写 out/run_summary.csv)
cargo run --release -j 10 -- det   # 全矩阵双跑,逐行比较状态哈希(out/run_det.txt)
python scripts/summary.py          # 从提交数据算 README 数字
```

**提交的原始数据**(默认运行不覆盖;`out/*` gitignore,只有这三个白名单):
`out/matrix_submitted.txt`(420 行全表)、`out/det_submitted.txt`(420 对)、
`out/eight_per_robot_submitted.csv`(8 只组逐只行)。独立 `target` 已 gitignore。

## R1 修正的 M1–M10(逐条)

| # | 验收问题 | 修正 |
|---|---|---|
| M1 | `restore_robot` 不写目标,主体失力 1–2 拍 | `quad::restore_robot` 经 `set_target` 同一路径把 `snap.targets` 写回(生产 30 Hz 同参 kp/kd/tau);全部迁移/唤醒行受益 |
| M2 | 迁移拍 `continue` 不步进、不写目标 | 迁移分支只做快照→移除→重建,当拍照常写目标、照常步进 |
| M3 | 窗口混入出生段 | `post_*` 只统计迁移拍起 POST_TICKS=120 拍;对照组窗口基点 = 被对照场景的迁移拍(同相位对齐) |
| M4 | `post_link_max/post_pen_max` 是全程值 | 与窗口统计合并(WinStats 只收窗口内拍),列名与内容一致 |
| M5 | 摔倒只在窗口内检查 | 从迁移/对照基点起全程检查(land 组不判摔,窗口覆盖落地冲击,只报数) |
| M6 | 缺"机身相邻拍位移" | 新增 `post_body_step_max` 列(窗口内机身相邻拍位移最大值) |
| M7 | 行走对照只有 p06、站立对照没强制醒着 | 行走对照 12 相位各一条(T0/T1);站立对照 `control_stand_awake`(每拍唤醒)与 `control_stand_sleep`(睡眠口径)各一条 |
| M8 | 来回迁移拿 1320 拍对照 | `control_pingpong` 同 3600 拍(窗口基点 = 首次迁移拍 walk0+60) |
| M9 | `novec` 行与 `same_full` 比无意义 | `diverge_tick` 与**相同快照类型**的 `same` 行比(scene_key 含快照类型) |
| M10 | 8 只组写死 0 | 逐只报误差/窗口指标/vfwd(`eight_per_robot_submitted.csv` 8 行)+ 汇总行;新增 `eight_control_T0` 不迁移对照 |

## rapier API 口径(追加进 quad,实现依据)

- 关节坐标没有公开 setter(`link.joint.coords` 是 `pub(crate)`);唯一公开
  写入路径 = `Multibody::apply_displacements`(增量位移)+ `forward_kinematics(
  bodies, true)` + `update_rigid_bodies`(连杆位姿写回刚体);
- 广义速度走 `Multibody::generalized_velocity_mut`(下一拍 solver 的
  `update_velocities` 从它重建连杆速度),根刚体 `linvel/angvel` 同时直写;
- 恢复后逐位核对:`quad::measure_restore_error`(读回关节角与刚体位姿比较,
  不拿快照比快照);
- `quad::active_order`:主体连杆在 `islands.active_bodies()` 活跃集里的顺序
  (连杆顺序表用)。

## 矩阵(420 行)

行走迁移 12 相位 × 重建家族 6 × 快照 2 × 地形 2 = 288 + 站立醒/眠 × 24 +
落地 24 + pingpong 6 + 对照 32 + order 19 + seq 1 + 8 只组 2(scripts/
summary.py 的行数计数)。

- 重建:`same`(同世界移除再重建)/ `cross`(移到第二个世界,两世界同拍
  步进)/ `cross_hist`(cross + 句柄历史 + 20 m 外 3×3×3 堆)/ **R1-5 拆分**:
  `histonly`(只有句柄历史)、`pilesleep`(只有方块堆,迁移拍堆已睡着)、
  `pileawake`(只有方块堆,迁移拍前每拍唤醒);
- 快照:`full` / `novec`(根速度与关节速度清零);
- 地形:T0 大平面;T1 chunk 接缝(`greedy_colliders`,与 terrain 同构)。

## 判据逐条(R1 口径;通过与否由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | 完整快照重建后、步进前:关节坐标 < 1e-4 rad、连杆位置 ≤ 1e-4 m | ✅ 279 个 full 行误差全部逐位为 0(0.000e0;summary.py 判据1) |
| 2 | 完整快照迁移后 120 拍真实窗口:up·y ≥ 0.97、\|vy\| ≤ 0.35、连杆 ≤ max(20, 2×同相位对照)、机身相邻拍位移 ≤ 1.5×对照 | ✅ 行走/站立 261 行 full:up·y min 0.9888、\|vy\| max 0.2844、连杆 2.0767(同相位对照 2.0175)、body_step_max 0.0107(= 对照同值);land 组按"各项不差于对照的 110%"逐项比:up·y 0.7500–0.7624 vs 对照 0.7459–0.7661、\|vy\| 3.4503–3.5561 vs 3.4543–3.4545(迁移组不高于对照 +3%,差值来自 novec 口径,full 行 3.4503–3.4504 逐位不高于对照)、连杆 3.5228–3.6197 vs 3.7763–3.7968、body_step 0.0572–0.0590 vs 0.0572(比值 1.000–1.031,全在 110% 内)——迁移没有加剧落地冲击 |
| 3 | 单次迁移 `same`/`cross`:全程不摔、vfwd ≥ 同相位对照 95%、窗口穿透 ≤ 5 cm;`cross_hist` 不计入单独报告 | ✅ same 99.9–100.7%、cross 84.1–100.7%(逐相位;最低的 cross 相位与 cross_hist 同机制,见 diverge 一节);0 摔倒(summary.py 摔倒行 = 无);行走窗口对地穿透最深 4.5 mm,land 28.7 mm(落地冲击本身,对照同量级);**`cross_hist` T0 56.3% / T1 84.1–84.5%,未过 95% 门槛——不计入判据 3(R1 裁定),单独报告见下** |
| 4 | 来回迁移间隔 ≥ 30 拍不摔、vfwd ≥ 同窗口对照 80%;更短间隔只报数 | ✅ g30 = 100.0%、g120 = 100.5%,0 摔倒;g5 = 94.1%、g2 = 69.2%(up·y 0.9915)只报数 |
| 5 | 全矩阵双跑逐行相同 | ✅ det 420 对全部 IDENTICAL(out/det_submitted.txt;summary.py det) |

**body_step_max 的口径说明**:行走/站立迁移组的窗口 body_step_max = 0.0107 m,
与同相位对照相同;全表最大值 1.1770 来自 `seq_shuffle4`(顺序打乱组,含
300 拍打乱段与重建拍,不是迁移窗口量级,见 order 一节);land 落地组的
body_step(0.0572–0.0590)是 0.5 m 落地冲击当拍的位移,对照同值(0.0572),
迁移不放大冲击(逐项比值 1.000–1.031)。

## `cross_hist` 现象(R1-5 拆分;连杆顺序)

`cross_hist` 不计入判据 3(R1 裁定),拆分后的数据(summary.py 判据3,相对
同相位 T0 对照 0.5161 / T1 0.5123):

| 重建家族 | T0 vfwd | T1 vfwd | 结论 |
|---|---|---|---|
| `histonly`(只有句柄历史) | 99.9–100.1% | 99.9–100.7% | 与 same 同水平——**单独的句柄历史没有影响** |
| `pilesleep`(堆已睡着) | 84.8–84.9% | 85.0–92.6% | 掉速与堆相关,不是句柄 |
| `pileawake`(堆醒着) | 77.1–92.7% | 92.5–94.1% | 同上,量级同 |
| `cross_hist`(两者都给) | 56.3% | 84.1–84.5% | 最差;堆 + 历史的组合 |

机制归因**见 #25 总架构师验收("连杆顺序"一节)**:刚体睡着时 island 用
`swap_remove` 移出活跃集,主体之前插入的刚体会把主体连杆搬到活跃集前面,
求解顺序不同,1 次 PGS 下足以改变行走偏置。本 demo 只报自己的数据:

- **order_* 表**(主体直接出生、不迁移;summary.py order;vfwd 对干净世界
  0.5161):之前插入 1 个方块 0.4326、2 个 0.4257、3 个 0.4375、4 个 0.3392、
  8 个 0.3014、13/16/27/64 个 0.2879–0.2880(≈16 个饱和,与探针 16/64 个
  0.2881/0.2882 同量级);之后插入 1/4/27 个 0.5161–0.5164(与对照同);
  64 个固定刚体 0.5163(不掉速);4/16 个无碰撞体刚体 0.3379/0.2878(与同
  数量方块一致);16 个出生就睡着的 0.5161(不掉速);方块堆 −200 m /
  +200 m 均 0.2880(与距离无关)。活跃集顺序:被插到主体之前的方块睡着后,
  主体连杆被搬到前面(end = `12-11-10-…-1-B` 完全倒序),start 列部分行是
  `none`(起步第一拍主体尚在睡眠未进活跃集,测量口径见 active_order 注释);
- **seq_*(顺序和重建的关系)**:4 方块打乱顺序(行走 300 拍 vfwd 0.2300、
  航向速率 −4.60 °/s)→ same 迁移 → 再走 1020 拍:迁移后顺序**复位**
  (`B-1-2-…-12`,重建让主体连杆回到活跃集尾部)、vfwd 回到 0.5400、航向
  速率 −0.00 °/s。**same 迁移把顺序打乱的影响清掉了**(重建 = 重新插入,
  主体排在最后);速度跟着恢复。

## `cross` 与 `same` 的轨迹分歧(diverge_tick,只报数不是判据)

与相同快照类型的 `same` 行逐拍哈希比(summary.py diverge):`cross_full`
分流行 16/24,分歧拍 193–901(迁移拍 180–198 之后);`cross_hist`/
`histonly`/`pilesleep`/`pileawake` 家族分流 176 行,分歧拍 180–902(迁移后
1 拍起)。生产迁移的确定性定义由总架构师裁定(R1 验收已给:**同一世界历史
+ 同一事件序列,重放逐位一致**;不承诺迁移与不迁移、或目标世界历史不同的
轨迹相同)。

## novec(不带速度)对照

vfwd 均值与 full 比:T0 99.9%、T1 100.2%(summary.py novec)——步行迁移
场景速度几乎不必带;落地/高速场景未单独验证,如实写。

## 耗时与其他

- 单次迁移(restore_robot):55.4–151.3 µs(summary.py migrate_us);
- 8 只同拍迁移:平均 60.1 µs/只;迁移后全体窗口 up·y min 0.9888、\|vy\| max
  0.1841、连杆 2.0175、body_step 0.0107、vfwd 0.5181(八只组逐只行 =
  `out/eight_per_robot_submitted.csv`,逐只数值与汇总一致);
- 睡眠迁移:已睡眠站立迁移后窗口 up·y ≥ 0.9926(matrix_submitted.txt
  stand_asleep 行,summary.py 判据2 口径覆盖)。

## 范围

只动 `demos/agents/migrate/`(代码 + 提交数据 + scripts/)与 `quad` 追加
(`restore_robot` 写回 targets 属 M1 修正,放行允许;`active_order` 新函数);
`quad` 追加后 `gait` 168 行、`terrain` 85 行、`mass` 5313 行状态哈希与提交
数据逐位相同(全量重跑比对,计时列除外)。不改 `crates/`、`docs/`。
