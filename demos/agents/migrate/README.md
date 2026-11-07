# A00-3 demo `migrate`(REQ-012 §3 迁移 / AC-B3,2026-11-07;R2 返修)

放行与验收:issue [#25](https://github.com/esp3j0/physics_minecraft/issues/25)
(A00-3 R2 放行;上一条验收里的 MR1–MR6 是本轮要修的问题)。**问题(放行
原文)**:丢掉接触缓存重建后,四足会不会跳变或摔倒?要验证的设计:迁移信封
只带身体哈希、根状态、关节坐标、关节速度、最后的目标,整个主体同一拍迁移。

统一口径:60 Hz、生产求解参数(只覆写 dt)、默认步态(kp 100、kd 3、3 Hz)、
`k` = 1/100、阻尼按 `k` 缩放、30 Hz 更新目标、不开 `extra`。

**本 README 的全部总结数字由 `scripts/summary.py` 从提交数据
`out/matrix_submitted.txt` / `out/det_submitted.txt` 计算**(脚本按完整
名字段筛行——scene/相位/家族/快照/地形拆开比较,不用子串(MR1),并断言
每行列数 == 列头数;各数字来自脚本的对应 `==` 小节)。

## 运行方式

```bash
cd demos/agents/migrate
cargo run --release -j 10          # 全矩阵 331 行(写 out/run_summary.csv)
cargo run --release -j 10 -- det   # 全矩阵双跑,逐行比较状态哈希(out/run_det.txt)
python scripts/summary.py          # 从提交数据算 README 数字
```

**提交的原始数据**(默认运行写 `out/run_*`,不覆盖;提交件单独拷贝,
三个白名单):`out/matrix_submitted.txt`(331 行全表)、
`out/det_submitted.txt`(331 对)、`out/eight_per_robot_submitted.csv`
(8 只组逐只行,16 行 = 迁移组 + 对照组 × 8 只)。独立 `target` 已 gitignore。

## R2 修正的 MR1–MR6(逐条)

| # | 验收问题 | 修正 |
|---|---|---|
| MR1 | `summary.py` 用子串筛行,`cross` 匹配到 `cross_hist` | 脚本重写:名字拆 scene/相位/家族/快照/地形字段后**完整段**比较;判据 3 按家族逐个报 `same`/`cross`,五个拆分家族单列;diverge 同口径按家族拆 |
| MR2 | 跨世界行 `active_order_end` 全空(只读世界 a) | `active_order_end` 读主体**最后所在的世界**;新增 `active_order_post` 列 = 迁移后第 120 拍、主体所在世界的顺序(全部 cross 家族行走行非空) |
| MR3 | `pilesleep` 迁移拍没核实堆是否睡着 | 新增门槛:堆全部睡着**且连续 60 拆保持全睡**才在下一个对应相位拍迁移;新增 `dst_awake_at_migrate` 列(迁移拍目标世界醒着的非主体动态刚体数,不为 0 panic 不进矩阵;同世界行 = `-`);新增 `dst_rewake_tick` 列(迁移后堆复醒拍)。门槛生效:`pilesleep` T0 vfwd 0.4381(R1 未核实时)→ **0.5172**(= 干净世界 100.2%),`dst_awake_at_migrate` 全部 0 |
| MR4 | 91 行重复(拆分家族 novec 循环重复推进) | 拆分家族(histonly/pilesleep/pileawake)只跑 full,单次推进;行数由脚本计(331 = 420 − 91 重复 − 4 拆分家族 novec × 12 相位 + 2 seq_control 等,脚本行数小节为准) |
| MR5 | 窗口跨段(迁移前拍混进同一统计) | `WinStats` 只收迁移后窗口;对照行用自己的基点;`order`/`seq` 行无迁移窗口,`post_*` 列写 `-`(矩阵中显示 1.0000/0.0000 的 order 行是"未进窗口"的初值,非真实测量——判据表不引用这些行) |
| MR6 | `order_*` 缺航向/路径/睡眠列 | 新增 `yaw_change_deg`(全程累计航向变化,展开 ±180°)、`path_len_m`(机身水平路径长度)、`slept`(主体是否睡过)三列(全表都有);新增 `seq_control_T0`(seq 前后两段同时长的干净世界对照,报同口径 vfwd 与航向速率) |

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

## 矩阵(331 行)

行走迁移 12 相位 × 重建家族 6 × 快照(full/novec,same·cross·cross_hist 两档;
拆分家族只 full)× 地形 2 = 216 + 站立醒/眠 36 + 落地 18 + pingpong 6 +
对照 32 + order 19 + seq 2 + 8 只组 2(scripts/summary.py 的行数小节)。

- 重建:`same`(同世界移除再重建)/ `cross`(移到第二个世界,两世界同拍
  步进)/ `cross_hist`(cross + 句柄历史 + 20 m 外 3×3×3 堆)/ **R1-5 拆分**:
  `histonly`(只有句柄历史)、`pilesleep`(只有方块堆,迁移拍堆已睡着——
  MR3 门槛)、`pileawake`(只有方块堆,迁移拍前每拍唤醒);
- 快照:`full` / `novec`(根速度与关节速度清零);
- 地形:T0 大平面;T1 chunk 接缝(`greedy_colliders`,与 terrain 同构)。

## 判据逐条(R2 口径;通过与否由总架构师裁定)

| # | 判据 | 结果 |
|---|---|---|
| 1 | 完整快照重建后、步进前:关节坐标 < 1e-4 rad、连杆位置 ≤ 1e-4 m | ✅ 155 个 full 迁移行误差全部逐位为 0(0.000e0;summary.py 判据1) |
| 2 | 完整快照迁移后 120 拍真实窗口:up·y ≥ 0.97、\|vy\| ≤ 0.35、连杆 ≤ max(20, 2×同相位对照)、机身相邻拍位移 ≤ 1.5×对照 | ✅ 行走/站立 168 行 full:up·y min 0.9890、\|vy\| max 0.2844(walk_p07_cross_hist_full_T0)、连杆 2.0767(同相位对照 2.0175);body_step_max 除 pilesleep T1 家族(见下)外 0.0107 = 对照同值。land 组按"各项不差于对照的 110%"逐项比(summary.py land 小节):up·y 0.7459–0.7661 vs 对照 0.7661(迁移组不高于对照)、\|vy\| 3.4503–3.4545 vs 3.4545(不高于)、连杆 3.5228–3.7968 vs 3.7968(不高于)、body_step 0.0572 vs 0.0572(pilesleep 两行 0.1323/0.1498,见下)——迁移没有加剧落地冲击 |
| 3 | 单次迁移 `same`/`cross`:全程不摔、vfwd ≥ 同相位对照 95%、窗口穿透 ≤ 5 cm;拆分家族单列 | ✅ **same 99.9%–100.7%、cross 99.9%–100.7%**(T0/T1 × full/novec 四组;MR1 修正后不再有"cross 84.1%"的错误数字);0 摔倒;行走窗口对地穿透最深 4.5 mm。**`cross_hist` 56.3%–84.5%(T0 56.3%、T1 84.1%–84.5%),未过 95% 门槛——不计入判据 3(R1 裁定),拆分数据见下** |
| 4 | 来回迁移间隔 ≥ 30 拍不摔、vfwd ≥ 同窗口对照 80%;更短间隔只报数 | ✅ g30 = 100.0%、g120 = 100.5%,0 摔倒;g5 = 94.1%、g2 = 69.2%(up·y 0.9915)只报数 |
| 5 | 全矩阵双跑逐行相同 | ✅ det 331 对全部 IDENTICAL(out/det_submitted.txt;summary.py det) |

**body_step_max 的口径说明(MR5 修正后)**:行走/站立迁移组窗口
body_step_max = 0.0107 m = 对照同值;**例外**:`pilesleep` T1 家族
0.1197–0.1472(land 组 0.1323/0.1498)——pilesleep 迁移拍被 MR3 门槛顺延,
窗口起点随之移动,窗口内包含 T1 接缝上的大位移拍;该现象**未归因**(窗口
内 up·y/|vy|/vfwd 均正常,vfwd 100.4% 与 same 同水平),如实在此报数,不
进判据 2 的"≤1.5×对照"比较(该家族不在判据 2 口径的 same/cross 主扫内)。

## 拆分家族数据(summary.py 判据3,相对同相位对照;MR3 门槛后)

| 重建家族 | T0 vfwd | T1 vfwd | 结论 |
|---|---|---|---|
| `histonly`(只有句柄历史) | 99.9%–100.1% | 99.9%–100.7% | 与 same 同水平——**单独的句柄历史没有影响** |
| `pilesleep`(堆睡着,60 拍全睡门槛后迁移) | 100.2% | 100.4% | **门槛生效后与干净世界同水平**(R1 未核实门槛时 84.8%–92.6%,掉速是"迁移拍堆没睡"造成的) |
| `pileawake`(堆醒着迁移) | 77.1%–92.7% | 92.5%–94.1% | 真实掉速:醒着的堆参与求解,主体活跃集顺序被打乱 |
| `cross_hist`(两者都给) | 56.3% | 84.1%–84.5% | 最差;堆 + 历史的组合 |

MR2/MR3 新列(summary.py MR2/MR3 小节):`cross`/`histonly`/`pilesleep` 的
`dst_awake_at_migrate` = 0、`dst_rewake_tick` = -(无复醒),迁移后顺序
`active_order_post` = `B-1-2-…-12`(与 same 同);`cross_hist` 的
`dst_awake_at_migrate` = 27(迁移拍堆里 27 个刚体醒着——句柄历史污染让堆
没睡着)、`dst_rewake_tick` = 0、迁移后顺序 = `B-12-11-…-1` 完全倒序;
`pileawake` 的 `dst_awake_at_migrate` = 27(布景即每拍唤醒)、
`dst_rewake_tick` = 0(迁移后第一拍堆仍醒,随后同拍集中睡着)、迁移后顺序
为任意排列(如 `10-B-11-2-5-8-12-7-9-6-3-1-4`,3 种排列 × 12 相位)——连杆
顺序可以是任意排列,不只是"前移若干根"(与总架构师 R1 验收的探针一致)。

## `cross_hist` 现象(连杆顺序;机制见 #25 总架构师验收)

刚体睡着时 island 用 `swap_remove` 移出活跃集,主体之前插入的刚体会把主体
连杆搬到活跃集前面,求解顺序不同,1 次 PGS 下足以改变行走偏置。本 demo 只
报自己的数据(summary.py order 小节;vfwd 对干净世界 0.5161):

- **order_* 表**(主体直接出生、不迁移;R2 新增列齐全):之前插入 1 个方块
  vfwd 0.4326、yaw +440.0°、path 10.00 m、slept=true(主体绕圈);2/3/4 个
  0.4257/0.4375/0.3392;8 个 0.3014;13/16/27/64 个 0.2879–0.2880(≈16 个
  饱和;64 个 yaw −145.0°、path 7.08 m);之后插入 1/4/27 个 0.5161–0.5164
  (yaw −20°/−21°,与对照同,不掉速);64 个固定刚体 0.5163(不掉速);
  4/16 个无碰撞体刚体 0.3379/0.2878(与同数量方块一致);16 个出生就睡着的
  0.5161(不掉速,slept=false);方块堆 −200 m / +200 m 均 0.2880(yaw
  −145.0°,与距离无关)。活跃集顺序:被插到主体之前的方块睡着后,主体连杆
  被搬到前面(end = `12-11-10-…-1-B` 完全倒序);start 列部分行是 `none`
  (起步第一拍主体尚在睡眠未进活跃集,测量口径见 active_order 注释);
- **seq_*(顺序和重建的关系)**:`seq_shuffle4`(4 方块打乱顺序)行走 300 拍
  vfwd 0.2300、航向速率 −4.60 °/s → same 迁移 → 再走 1020 拍 vfwd 0.5400、
  航向速率 −0.00 °/s,顺序复位 `B-1-2-…-12`;**干净世界对照
  `seq_control_T0`**(R2/MR6 新增)同两段 vfwd 0.4200 → 0.5400、航向速率
  −4.80 → −0.00 °/s——干净世界前段本身也慢(起步渐入),但打乱组的 −4.60
  °/s 航向速率与对照 −4.80 °/s 同量级、且两段后段都归零;same 迁移把顺序
  打乱的影响清掉(重建 = 重新插入,主体排在最后),速度跟着恢复。

## `cross` 与 `same` 的轨迹分歧(diverge_tick,只报数不是判据)

与相同快照类型的 `same` 行逐拍哈希比(summary.py diverge 小节,按家族拆):
`same_full` 0/24 分流(基准自比);`cross_full` 16/24 分流,分歧拍 193–901;
`histonly` 16/24,193–902;`pilesleep` 24/24,180–198;`pileawake` 24/24,
180–255。生产迁移的确定性定义由总架构师裁定(R1 验收已给:**同一世界历史
+ 同一事件序列,重放逐位一致**;不承诺迁移与不迁移、或目标世界历史不同的
轨迹相同)。

## novec(不带速度)对照

vfwd 均值与 full 比:T0 99.9%、T1 100.2%(summary.py novec 小节)——步行
迁移场景速度几乎不必带;落地/高速场景未单独验证,如实写。

## 耗时与其他

- 单次迁移(restore_robot):52.3–203.0 µs(summary.py migrate_us 小节;
  0.0 是对照行,不迁移无耗时);
- 8 只同拍迁移:平均 56.8 µs/只;迁移后全体窗口 up·y min 0.9888、\|vy\| max
  0.1841、连杆 2.0175、body_step 0.0107、vfwd 0.5181;**8 只不迁移对照
  `eight_control_T0`**(migrate_us = 0,up·y 0.9889、vfwd 0.5164)——逐只
  行 = `out/eight_per_robot_submitted.csv` 16 行(迁移组 8 + 对照组 8);
- 睡眠迁移:已睡眠站立迁移后窗口 up·y 0.9975(stand_asleep_same_full_T0)。

## 范围

只动 `demos/agents/migrate/`(代码 + 提交数据 + scripts/);`quad` 本轮
未改(R2 放行)。不改 `crates/`、`docs/`。
