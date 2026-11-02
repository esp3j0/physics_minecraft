//! compound_flow 单元测试(AC-G1 惯性汇点网络、AC-G2 动量分配)。
//!
//! s* 锚点数值与 S 形状全部来自 G00 `flow_out.txt`(2026-11-02 存档;
//! §5.8.1 表纯函数列,d570a24 回写)。断言口径:s* − 0.005 不断、
//! s* + 0.005 断(量化精度 0.001 N·s;±0.002 内存在"ceil(d_i)+粘结容量恰
//! 等 supply"的 tie 带,任何整化实现都可能在带内翻转,不作为断言口径)。
//! 作用点一律取格面
//! (格心沿 −normal 偏 0.5,同 G00 flow.py 的 p_side / p_pillar_bottom /
//! p_axial / p_end_corner 口径,不是 collider 面 ±0.45)。

use types::BlockPos;
use types::glam::Vec3;
use voxel::{SimpleRegistry, builtin};

use super::*;

fn p(x: i32, y: i32, z: i32) -> BlockPos {
    BlockPos::new(x, y, z)
}

fn stone_event(cells: &[BlockPos], contacts: Vec<ContactPoint>) -> CompoundEvent {
    CompoundEvent {
        cells: cells.iter().map(|&c| (c, builtin::STONE)).collect(),
        contacts,
        linvel: Vec3::ZERO,
        angvel: Vec3::ZERO,
        correction: Vec3::ZERO,
    }
}

fn as_dirt(mut ev: CompoundEvent) -> CompoundEvent {
    for (_c, b) in &mut ev.cells {
        *b = builtin::DIRT;
    }
    ev
}

/// 单接触点事件:打 `hit` 的 `normal` 一侧(法向 = 指向 B 内部的单位向量),
/// 作用点取格面(格心沿 −normal 偏 0.5)。
fn hit_event(cells: &[BlockPos], hit: BlockPos, normal: Vec3, s: f32) -> CompoundEvent {
    let center = Vec3::new(hit.x as f32 + 0.5, hit.y as f32 + 0.5, hit.z as f32 + 0.5);
    stone_event(
        cells,
        vec![ContactPoint {
            cell: hit,
            impulse: s,
            point: center - normal * 0.5,
            normal,
        }],
    )
}

/// 平落事件:全部格接触、均摊,J = +y 指向条内部(地面冲量向上),
/// 作用点 = 各格底面。
fn flat_drop(cells: &[BlockPos], s: f32) -> CompoundEvent {
    let contacts: Vec<ContactPoint> = cells
        .iter()
        .map(|&c| ContactPoint {
            cell: c,
            impulse: s / cells.len() as f32,
            point: Vec3::new(c.x as f32 + 0.5, c.y as f32 - 0.5, c.z as f32 + 0.5),
            normal: Vec3::Y,
        })
        .collect();
    stone_event(cells, contacts)
}

/// 把事件冲量总量换成 s(方向不变;多接触点按比例缩放)。
fn scaled(ev: &CompoundEvent, s: f32) -> CompoundEvent {
    let mut out = ev.clone();
    let cur: f32 = out.contacts.iter().map(|c| c.impulse).sum();
    let k = s / cur;
    for pt in &mut out.contacts {
        pt.impulse *= k;
    }
    out
}

fn beam() -> Vec<BlockPos> {
    (0..6).map(|x| p(x, 0, 0)).collect()
}

fn pillar() -> Vec<BlockPos> {
    (0..6).map(|y| p(0, y, 0)).collect()
}

fn plate() -> Vec<BlockPos> {
    let mut v: Vec<BlockPos> = (0..3)
        .flat_map(|x| (0..3).map(move |z| p(x, 0, z)))
        .collect();
    v.sort();
    v
}

fn cube() -> Vec<BlockPos> {
    let mut v: Vec<BlockPos> = (0..2)
        .flat_map(|x| (0..2).flat_map(move |y| (0..2).map(move |z| p(x, y, z))))
        .collect();
    v.sort();
    v
}

/// 阈值断言:s* − 0.005 不断、s* + 0.005 断(±0.002 内是 tie 带,见模块注释)。
fn assert_threshold(reg: &SimpleRegistry, ev: &CompoundEvent, s_star: f32) {
    let lo = evaluate_compound_flow(reg, &scaled(ev, s_star - 0.005));
    assert!(
        lo.detached.is_empty(),
        "s* − 0.005 应不断:detached = {:?}",
        lo.detached
    );
    let hi = evaluate_compound_flow(reg, &scaled(ev, s_star + 0.005));
    assert!(
        !hi.detached.is_empty(),
        "s* + 0.005 应断(实际扛住,C = {})",
        hi.cut_q
    );
}

// ---------------------------------------------------------------- AC-G1 阈值

/// 工况 4 侧撞(+z,垂直于条轴线):端格 s* = 36.589(S = 端格、断 1 根)、
/// 第 2 格 50.675(S = 被撞格单独、断 2 根)、第 3 格 40.680(S = 一侧 4 格、
/// 断 1 根);土 14.232 / 19.709 / 15.825。变异红:去 max(0,·)、惯量 /12。
#[test]
fn acg1_side_hit_thresholds() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    for (i, s_star) in [(0, 36.589_f32), (1, 50.675), (2, 40.680)] {
        let ev = hit_event(&cells, cells[i], Vec3::Z, 1.0);
        assert_threshold(&reg, &ev, s_star);
    }
    let end = evaluate_compound_flow(
        &reg,
        &scaled(&hit_event(&cells, cells[0], Vec3::Z, 1.0), 36.599),
    );
    assert_eq!(end.detached, vec![p(0, 0, 0)], "侧撞端格 S = 端格");
    assert_eq!(end.cut_bonds, 1);
    let mid = evaluate_compound_flow(
        &reg,
        &scaled(&hit_event(&cells, cells[1], Vec3::Z, 1.0), 50.685),
    );
    assert_eq!(mid.detached, vec![p(1, 0, 0)], "侧撞第 2 格 S = 被撞格单独");
    assert_eq!(mid.cut_bonds, 2, "第 2 格断 2 根");
    let third = evaluate_compound_flow(
        &reg,
        &scaled(&hit_event(&cells, cells[2], Vec3::Z, 1.0), 40.690),
    );
    assert_eq!(
        third.detached,
        vec![p(2, 0, 0), p(3, 0, 0), p(4, 0, 0), p(5, 0, 0)],
        "侧撞第 3 格 S = 被撞格及远侧 4 格(2 + 4;Δω 使远端 d_i 更小,远侧割先饱和)"
    );
    assert_eq!(third.cut_bonds, 1);
    for (i, s_star) in [(0, 14.232_f32), (1, 19.709), (2, 15.825)] {
        let ev = as_dirt(hit_event(&cells, cells[i], Vec3::Z, 1.0));
        assert_threshold(&reg, &ev, s_star);
    }
}

/// 工况 3 竖柱(+y 打底格底面,p = 格面):底→上粘结为压向,s* = 86.402(石,
/// 压向割 I_c = 72)/ 33.602(土);S = 底格。
/// 变异红:地面冲量方向取反(压向判定翻转,割变 18/7)。
#[test]
fn acg1_pillar_crush_threshold() {
    let reg = SimpleRegistry::builtin();
    let cells = pillar();
    let ev = hit_event(&cells, p(0, 0, 0), Vec3::Y, 1.0);
    assert_threshold(&reg, &ev, 86.402);
    let brk = evaluate_compound_flow(&reg, &scaled(&ev, 86.412));
    assert_eq!(brk.detached, vec![p(0, 0, 0)], "竖柱 S = 底格");
    // 割 = 底格的汇边(d_i = m·Δv·d̂ ≈ s/6)+ 底格向上的压向粘结 72
    // (合计 ≈ 86.4,与 s 同量级——割恰在底格自身吸收 + 粘结带上)。
    assert_eq!(brk.cut_q, 86_403);
    let dirt_ev = as_dirt(hit_event(&cells, p(0, 0, 0), Vec3::Y, 1.0));
    assert_threshold(&reg, &dirt_ev, 33.602);
}

/// 工况 4b 轴向(+x 打端格 −x 面,p = 格面):压向口径同竖柱,
/// s* = 86.402 / 33.602(§5.8.1 表同值)。
#[test]
fn acg1_axial_hit_threshold() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let ev = hit_event(&cells, p(0, 0, 0), Vec3::X, 1.0);
    assert_threshold(&reg, &ev, 86.402);
    let dirt_ev = as_dirt(hit_event(&cells, p(0, 0, 0), Vec3::X, 1.0));
    assert_threshold(&reg, &dirt_ev, 33.602);
}

/// 工况 2a 端角:水平条端格外下角(p = 端格心 + (−0.5, −0.5, 0),J = +y):
/// s* = 42.484(石)/ 16.524(土);S = 端格、断 1 根。
#[test]
fn acg1_end_corner_threshold() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let mut ev = hit_event(&cells, p(0, 0, 0), Vec3::Y, 1.0);
    ev.contacts[0].point += Vec3::new(-0.5, -0.5, 0.0);
    assert_threshold(&reg, &ev, 42.484);
    let brk = evaluate_compound_flow(&reg, &scaled(&ev, 42.486));
    assert_eq!(brk.detached, vec![p(0, 0, 0)]);
    assert_eq!(brk.cut_bonds, 1);
    let dirt_ev = as_dirt(ev);
    assert_threshold(&reg, &dirt_ev, 16.524);
}

/// 工况 1/5/7 平落(全部格接触、均摊):石、土在 s ≤ 1e5 都不断。
/// 变异红:供给改逐点分配(平落的 d_i 均匀,逐点集中供给会假断)。
#[test]
fn acg1_flat_drop_never_breaks() {
    let reg = SimpleRegistry::builtin();
    for (name, cells) in [("beam", beam()), ("plate", plate()), ("cube", cube())] {
        for block in [builtin::STONE, builtin::DIRT] {
            let mut ev = flat_drop(&cells, 1.0);
            for (_c, b) in &mut ev.cells {
                *b = block;
            }
            let out = evaluate_compound_flow(&reg, &scaled(&ev, 100_000.0));
            assert!(
                out.detached.is_empty(),
                "{name} 平落 1e5 不断(detached = {:?})",
                out.detached
            );
        }
    }
}

/// §5.7 撞击者修正:`J` 减去 correction 后再判定(punch 口径——扣除前断、
/// 扣除后扛住)。
#[test]
fn correction_subtracted_from_j() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let mut ev = scaled(&hit_event(&cells, cells[0], Vec3::Z, 1.0), 50.0);
    // 修正冲量沿 +z(对方把 J 抵消到 40.0 < 50.675 第 2 格口径;对端格 36.589
    // 用 20.0 抵到 30.0)。
    ev.correction = Vec3::new(0.0, 0.0, 20.0);
    let out = evaluate_compound_flow(&reg, &ev);
    assert!(out.detached.is_empty(), "扣除后 s = 30 < 36.589 应扛住");
    let out2 = evaluate_compound_flow(&reg, &{
        let mut e = ev.clone();
        e.correction = Vec3::ZERO;
        e
    });
    assert!(!out2.detached.is_empty(), "不扣除 s = 50 > 36.589 应断");
}

/// 预筛:s < I_min 不建网(supply_q = cut_q = 0、prefiltered);s ≥ I_min 不命中。
#[test]
fn acg1_prefilter() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    // I_min = 4(sand);s = 3.999 命中。
    let ev = hit_event(&cells, p(0, 0, 0), Vec3::Z, 1.0);
    let out = evaluate_compound_flow(&reg, &scaled(&ev, 3.999));
    assert!(out.prefiltered);
    assert_eq!(out.supply_q, 0);
    assert_eq!(out.cut_q, 0);
    assert!(out.detached.is_empty());
    // s = 4.001 不命中(石条割 18,扛住但不预筛)。
    let out = evaluate_compound_flow(&reg, &scaled(&ev, 4.001));
    assert!(!out.prefiltered);
    assert!(out.detached.is_empty(), "4.001 < 18 扛住");
}

/// 惯性容量的取正语义 `d_i = m_i·max(0, Δv_i·d̂)`(§5.8.1):端角工况
/// (p = 端格外下角,Δω ≠ 0)使远端格沿 −d̂ 运动(Δv_i·d̂ < 0,python demo
/// 实测格 4/5 为 −6.5/−20.0),这些格的汇边容量必须为 0(旋转致"拉"不吸收,
/// 标量流已知近似);判定与 demo 一致(s = 60 断端格)。
/// 注意:`quant_cap` 对 ≤ 0 一律 0,与 max(0,·) 在求值结果上等价——本测试
/// 钉住公式语义(远端格不入汇),防止实现把 d_i 改为未经取正的口径。
#[test]
fn acg1_negative_inertia_sink_is_zero() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let mut ev = hit_event(&cells, p(0, 0, 0), Vec3::Y, 1.0);
    ev.contacts[0].point += Vec3::new(-0.5, -0.5, 0.0);
    // Δω ≠ 0:作用点偏心。
    let j = ev.impulse_j();
    let props = body_props(&cells);
    let (_, dv) = rigid_response(&props, &cells, ev.weighted_point(), j);
    let d_hat = ev.dir_hat();
    let neg_cells: Vec<BlockPos> = cells
        .iter()
        .filter(|&&c| dv[&c].dot(d_hat) < 0.0)
        .copied()
        .collect();
    assert_eq!(neg_cells.len(), 2, "端角 60 N·s 下应有 2 个负投影格");
    // 判定与手工期望一致:s = 60 断(S = 端格,负投影格 ∈ R)。
    let out = evaluate_compound_flow(&reg, &scaled(&ev, 60.0));
    assert_eq!(out.detached, vec![p(0, 0, 0)]);
    assert!(
        !out.detached.iter().any(|c| neg_cells.contains(c)),
        "负投影格(零汇容量)不该断进 S"
    );
}

/// 接触点顺序打乱(全部 6 个排列;demo 20 次随机打乱的确定性等价):
/// 判定逐位相同。
#[test]
fn acg1_contact_shuffle_invariant() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let pts = [
        (p(0, 0, 0), 30.0_f32, Vec3::new(0.5, 0.5, 0.0)),
        (p(1, 0, 0), 20.0, Vec3::new(1.5, 0.5, 0.0)),
        (p(2, 0, 0), 10.0, Vec3::new(2.5, 0.5, 0.0)),
    ];
    let make = |order: &[usize]| {
        let contacts = order
            .iter()
            .map(|&i| ContactPoint {
                cell: pts[i].0,
                impulse: pts[i].1,
                point: pts[i].2,
                normal: Vec3::Z,
            })
            .collect();
        stone_event(&cells, contacts)
    };
    let base = evaluate_compound_flow(&reg, &make(&[0, 1, 2]));
    for perm in [
        [0usize, 1, 2],
        [2, 1, 0],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [0, 2, 1],
    ] {
        let out = evaluate_compound_flow(&reg, &make(&perm));
        assert_eq!(out, base, "打乱顺序改变判定");
    }
}

/// 随机扫 s(LCG 固定种子,禁止 rand 依赖):断开样本里 S 永不为全体、
/// S 与 s_components 一致。
#[test]
fn acg1_s_never_whole_body() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let ev = hit_event(&cells, p(0, 0, 0), Vec3::Z, 1.0);
    let mut state = 0x1234_5678_u64;
    let mut checked = 0;
    for _ in 0..200 {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let frac = (state >> 11) as f32 / (1_u64 << 53) as f32;
        let s = 10_f32.powf(0.3 + frac * 3.9);
        let out = evaluate_compound_flow(&reg, &scaled(&ev, s));
        if out.prefiltered || out.detached.is_empty() {
            continue;
        }
        checked += 1;
        assert_ne!(
            out.detached.len(),
            cells.len(),
            "s = {s}: S = 全体(Σd_i ≥ s 被破坏)"
        );
        assert_eq!(
            out.detached,
            out.s_components
                .iter()
                .flatten()
                .copied()
                .collect::<Vec<_>>(),
            "s_components 与 detached 不一致"
        );
    }
    assert!(checked > 10, "有效样本过少:{checked}");
}

/// ≤ 8 格工况与穷举所有含接触格的割比对:C 与 S 都相同(AC-G1 穷举割条;
/// 4 格条侧撞第 2 格,大 s 断开工况)。割容量以 evaluate 的同一口径手算:
/// 割 = 不在 X 的接触格源边 + X→非X 的格间边 + 在 X 的格的汇边(汇边被割
/// = 源侧格的惯性吸收也被切;最大流视角下"X 的出边"含汇边)。穷举 2⁴ 源侧
/// 集合取最小。
#[test]
fn acg1_bruteforce_min_cut_4cell() {
    let reg = SimpleRegistry::builtin();
    let cells: Vec<BlockPos> = (0..4).map(|x| p(x, 0, 0)).collect();
    let ev = scaled(&hit_event(&cells, cells[1], Vec3::Z, 1.0), 60.0);
    let out = evaluate_compound_flow(&reg, &ev);
    assert!(!out.detached.is_empty(), "60 > 阈值应断");
    let block_of: BTreeMap<BlockPos, BlockId> =
        cells.iter().map(|&c| (c, builtin::STONE)).collect();
    let contacts: BTreeSet<BlockPos> = ev.contacts.iter().map(|pt| pt.cell).collect();
    let j = ev.impulse_j();
    let d_hat = ev.dir_hat();
    let s = j.length();
    let props = body_props(&cells);
    let (_, dv) = rigid_response(&props, &cells, ev.weighted_point(), j);
    let edge_cap = |a: BlockPos, b: BlockPos| -> u64 {
        let u = Vec3::new((b.x - a.x) as f32, (b.y - a.y) as f32, (b.z - a.z) as f32);
        let raw = if u.dot(d_hat) >= COS_PRESS {
            impact_crush_between(&reg, block_of[&a], block_of[&b])
        } else {
            impact_bond_between(&reg, block_of[&a], block_of[&b])
        };
        quant_cap(raw)
    };
    let sink_cap = |c: BlockPos| -> u64 { quant_cap(CELL_MASS * (dv[&c].dot(d_hat)).max(0.0)) };
    let s_each_q = quant_supply_f64(f64::from(s / contacts.len() as f32));
    let mut best = u64::MAX;
    for mask in 0..(1u32 << cells.len()) {
        let in_x: Vec<bool> = cells
            .iter()
            .enumerate()
            .map(|(i, _)| mask & (1 << i) != 0)
            .collect();
        if !cells
            .iter()
            .zip(&in_x)
            .any(|(c, x)| *x && contacts.contains(c))
        {
            continue; // 源侧必含接触格
        }
        let mut cap = 0_u64;
        for (i, &c) in cells.iter().enumerate() {
            if !in_x[i] && contacts.contains(&c) {
                cap += s_each_q; // 源边被割
            }
            if in_x[i] {
                cap += sink_cap(c); // 源侧格的汇边(惯性吸收出网)
                for (k, &b) in cells.iter().enumerate() {
                    if k != i && neighbor4(c, b) && !in_x[k] {
                        cap += edge_cap(c, b);
                    }
                }
            }
        }
        best = best.min(cap);
    }
    assert_eq!(
        out.cut_q, best,
        "C ≠ 穷举最小割(穷举 = {best},C = {})",
        out.cut_q
    );
}

/// 4 格相邻判定(穷举割用;曼哈顿距离 1)。
fn neighbor4(a: BlockPos, b: BlockPos) -> bool {
    (a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs() == 1
}

// ---------------------------------------------------------------- AC-G2

/// 守恒残差:侧撞第 3 格(s = 41 刚过 40.680 阈值)断成 4 + 2(S = 被撞格
/// 及远侧 4 格,R = 近侧 2 格,无动态对方 = 地形):Σ R 退还 + Σ 对方修正(0)
/// + Σ S 获得 = 0(量化误差内)。
#[test]
fn acg2_conservation_residual() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let ev = scaled(&hit_event(&cells, cells[2], Vec3::Z, 1.0), 41.0);
    let out = evaluate_compound_flow(&reg, &ev);
    assert_eq!(out.detached.len(), 4);
    let split = distribute_momentum(&reg, &ev, &out, &[]);
    assert!(
        split.residual <= 0.001 * ev.contacts.len() as f32 + 1e-4,
        "守恒残差 {} 超量化口径",
        split.residual
    );
    let sum_s: Vec3 = split.s_impulses.iter().copied().sum();
    let sum_r: Vec3 = split.r_refunds.iter().map(|(_, v)| *v).sum();
    assert!(
        (sum_r + sum_s).length() <= 1e-2,
        "R 退还 + S 获得 ≠ 0:{}",
        (sum_r + sum_s).length()
    );
    assert_eq!(split.r_components.len(), 1, "R = 近侧 2 格连通一块");
    assert_eq!(split.other_corrections.len(), 0, "地形对方无修正");
}

/// 解析校验(§5.8.2 末条):纯平移(p = 质心)时撞击者交出
/// `m·M_S/(m+M_S)·u` 的分配代数 = corr = m_eff/(m_eff + M_S)·E、
/// S 得 M_S/(m_eff + M_S)·E。合成工况直接构造 outcome(单格块 M_S)。
#[test]
fn acg2_analytic_pure_translation() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let props = body_props(&cells);
    let mut ev = scaled(&hit_event(&cells, cells[0], Vec3::Z, 1.0), 100.0);
    ev.contacts[0].point = props.center_of_mass; // J 过质心 → Δω = 0
    // 纯平移核验:所有格 Δv 相同。
    let (_, dv) = rigid_response(&props, &cells, ev.weighted_point(), ev.impulse_j());
    let base_dv = dv[&p(0, 0, 0)];
    for v in dv.values() {
        assert!((*v - base_dv).length() <= 1e-6, "纯平移被破坏");
    }
    // 合成 outcome:S = 端格(M_S = 0.729)、C = 18(合成校验的假想割)。
    let one = vec![p(0, 0, 0)];
    let outcome = CompoundFlowOutcome {
        supply_q: 100_000,
        cut_q: 18_000,
        detached: one.clone(),
        s_components: vec![one],
        cut_bonds: 0,
        prefiltered: false,
    };
    let others = [OtherBody {
        effective_mass: 0.729,
        impulse_share: 1.0,
    }];
    let split = distribute_momentum(&reg, &ev, &outcome, &others);
    let s = 100.0_f32;
    let e = (s - 18.0) / s * s; // |E| = ρ·s(J 沿 +z)
    let corr = split.other_corrections[0];
    assert!(
        (corr.length() - e * 0.5).abs() <= 0.01,
        "corr = {} ≠ m_eff/(m_eff+M_S)·E = {}",
        corr.length(),
        e * 0.5
    );
    assert!(
        (split.s_impulses[0].length() - e * 0.5).abs() <= 0.01,
        "S 获得 {} ≠ M_S/(m_eff+M_S)·E = {}",
        split.s_impulses[0].length(),
        e * 0.5
    );
    assert!(split.residual <= 1e-3);
}

/// 解析校验:对方 = 地形(无 others)→ S 恰好拿走全部超出冲量 E、R 只被
/// 断面粘结拖慢(R 退还 = −E,分摊按断面容量);q = 断面质心 = 两格中心
/// 连线中点。
#[test]
fn acg2_terrain_other_s_stopped_r_tugged() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let ev = scaled(&hit_event(&cells, cells[0], Vec3::Z, 1.0), 40.0);
    let out = evaluate_compound_flow(&reg, &ev);
    assert_eq!(out.detached, vec![p(0, 0, 0)]);
    let split = distribute_momentum(&reg, &ev, &out, &[]);
    assert!(split.other_corrections.is_empty(), "地形对方无修正");
    let s = ev.total_impulse();
    let e_mag = (s - out.cut_q as f32 / 1000.0) / s * s;
    assert!(
        (split.s_impulses[0].length() - e_mag).abs() <= 0.01,
        "S 获得 {} ≠ E {}",
        split.s_impulses[0].length(),
        e_mag
    );
    let sum_r: Vec3 = split.r_refunds.iter().map(|(_, v)| *v).sum();
    assert!((sum_r.length() - e_mag).abs() <= 0.01, "Σ R 退还 ≠ −E");
    assert!(
        (split.r_components[0].cut_capacity - 18.0).abs() <= 0.01,
        "断面 T = 端格-格1 石粘结 = 18"
    );
    let q = split.r_components[0].cut_point;
    assert!(
        (q - Vec3::new(1.0, 0.5, 0.5)).length() <= 1e-4,
        "q = 两格中心连线中点(1.0, 0.5, 0.5)"
    );
}

/// L 形不对称分量:Δω 与完整惯量解析一致(惯性积进账;只取对角的实现
/// 在此工况必红——诊断断言钉差异)。L = [(0,0,0),(1,0,0),(1,1,0),(1,2,0)],
/// 冲量 −x 打在顶格 +x 面中心(同 G00 flow.py 工况 6 受力几何)。
#[test]
fn acg2_lshape_full_inertia() {
    let cells = [p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(1, 2, 0)];
    let props = body_props(&cells);
    // L 在 xy 平面不对称 → I_xy ≠ 0。
    assert!(props.inertia[0][1] != 0.0, "L 形惯性积应为非零");
    // 冲量沿 −z 打在顶格 +z 面中心:力矩落在 xy 平面(torque = r × P 的
    // x、y 分量非零),I⁻¹ 经 I_xy 把两个分量混合——惯性积进账;面内冲量
    // (−x)的力矩纯沿 z,平面体的 I_zz 无惯性积贡献,对角近似恰好同值,
    // 测不到完整张量(G00 工况 6 的 J=−x 几何只对 rapier 对照有效)。
    let point = Vec3::new(1.5, 2.5, 1.0);
    let impulse = Vec3::new(0.0, 0.0, -40.0);
    let (domega, dv) = rigid_response(&props, &cells, point, impulse);
    // 手算:r = p − c;torque = r × P;Δω = I⁻¹·torque(完整张量解析逆)。
    let com = props.center_of_mass;
    let torque = (point - com).cross(impulse);
    assert!(
        torque.x.abs() > 1e-3 && torque.y.abs() > 1e-3,
        "xy 力矩激励"
    );
    let m = &props.inertia;
    let (a, b, c0, d, e0, f) = (m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2]);
    let c00 = d * f - e0 * e0;
    let c01 = c0 * e0 - b * f;
    let c02 = b * e0 - d * c0;
    let c11 = a * f - c0 * c0;
    let c12 = b * c0 - a * e0;
    let c22 = a * d - b * b;
    let inv = 1.0 / (a * c00 + b * c01 + c0 * c02);
    let expect = Vec3::new(
        (c00 * torque.x + c01 * torque.y + c02 * torque.z) * inv,
        (c01 * torque.x + c11 * torque.y + c12 * torque.z) * inv,
        (c02 * torque.x + c12 * torque.y + c22 * torque.z) * inv,
    );
    assert!(
        (domega - expect).length() <= 1e-5,
        "Δω {domega} ≠ 完整张量解析 {expect}"
    );
    // 对角近似(惯量积丢掉的实现)在此工况给出不同结果:钉住差异存在。
    let diag = Vec3::new(torque.x / m[0][0], torque.y / m[1][1], torque.z / m[2][2]);
    assert!(
        (domega - diag).length() > 1e-3,
        "完整张量与对角近似无差异(工况没测到惯性积)"
    );
    // Δv 场逐格核对:Δv_i = P/M + Δω × (x_i − c)。
    for &cell in &cells {
        let xi = Vec3::new(
            cell.x as f32 + 0.5,
            cell.y as f32 + 0.5,
            cell.z as f32 + 0.5,
        );
        let expect_dv = impulse / props.mass + domega.cross(xi - com);
        assert!((dv[&cell] - expect_dv).length() <= 1e-5);
    }
}

/// piece_velocity:多格块 = B 速度场(过块质心)+ 自身冲量响应;
/// 单格块丢角速度。
#[test]
fn piece_velocity_field_and_response() {
    let cells = beam();
    let body = body_props(&cells);
    let linvel = Vec3::new(1.0, 2.0, 3.0);
    let angvel = Vec3::new(0.1, 0.2, 0.3);
    let point = Vec3::new(0.5, 0.5, 0.5);
    let impulse = Vec3::new(0.0, 0.0, 30.0);
    // 多格块(R = 格 1..6):v = 场(块质心)+ P/M;ω = Δω ≠ 0(离轴)。
    let r_cells: Vec<BlockPos> = (1..6).map(|x| p(x, 0, 0)).collect();
    let (v, w) = piece_velocity(&r_cells, &body, linvel, angvel, point, impulse, false);
    let com = component_center_of_mass(&r_cells);
    let field = linvel + angvel.cross(com - body.center_of_mass);
    assert!((v - (field + impulse / (r_cells.len() as f32 * CELL_MASS))).length() <= 1e-5);
    assert!(w != Vec3::ZERO, "多格块保角速度");
    // 单格块:角速度置零,只有速度场。
    let one = [p(0, 0, 0)];
    let (v1, w1) = piece_velocity(&one, &body, linvel, angvel, point, impulse, true);
    let com1 = component_center_of_mass(&one);
    assert!((v1 - (linvel + angvel.cross(com1 - body.center_of_mass))).length() <= 1e-6);
    assert_eq!(w1, Vec3::ZERO);
}

/// 修正分配的冲量份额拆账:两个动态对方各占一半份额,修正按份额拆。
#[test]
fn other_correction_share_split() {
    let reg = SimpleRegistry::builtin();
    let cells = beam();
    let ev = scaled(&hit_event(&cells, cells[0], Vec3::Z, 1.0), 40.0);
    let out = evaluate_compound_flow(&reg, &ev);
    assert_eq!(out.detached, vec![p(0, 0, 0)]);
    let others = [
        OtherBody {
            effective_mass: 0.729,
            impulse_share: 0.5,
        },
        OtherBody {
            effective_mass: 2.0,
            impulse_share: 0.5,
        },
    ];
    let split = distribute_momentum(&reg, &ev, &out, &others);
    let s = ev.total_impulse();
    let e_mag = (s - out.cut_q as f32 / 1000.0) / s * s;
    let m_s = CELL_MASS;
    let want0 = e_mag * 0.5 * (0.729 / (0.729 + m_s));
    let want1 = e_mag * 0.5 * (2.0 / (2.0 + m_s));
    assert!((split.other_corrections[0].length() - want0).abs() <= 1e-3);
    assert!((split.other_corrections[1].length() - want1).abs() <= 1e-3);
    assert!(split.residual <= 1e-3);
}

// ------------------------------------------------- 代价(#[ignore],只报告)

/// 216 格单事件耗时(目标 < 0.2 ms;release 跑,只打印不上断言)。
/// 测试内测墙钟:`#[ignore]` 专属测量,不进门禁、不影响确定性(核纪律
/// clippy.toml 的 Instant 禁令针对生产路径;此处局部 allow 注明理由)。
#[test]
#[ignore = "代价测量:cargo test -p physics-sim --release cost_216 -- --ignored"]
fn cost_216_cells_release() {
    let reg = SimpleRegistry::builtin();
    let cells: Vec<BlockPos> = (0..6)
        .flat_map(|x| (0..6).flat_map(move |y| (0..6).map(move |z| p(x, y, z))))
        .collect();
    // 端面 36 格受击,冲量沿 +x 作用于端面质心(G00 case7 同布景)。
    let contacts: Vec<ContactPoint> = cells
        .iter()
        .filter(|c| c.x == 0)
        .map(|&c| ContactPoint {
            cell: c,
            impulse: 500.0 / 36.0,
            point: Vec3::new(0.0, c.y as f32 + 0.5, c.z as f32 + 0.5),
            normal: Vec3::X,
        })
        .collect();
    let ev = stone_event(&cells, contacts);
    let mut times = Vec::new();
    let mut last = CompoundFlowOutcome::default();
    for _ in 0..9 {
        #[allow(clippy::disallowed_methods)] // #[ignore] 专属测量,不进门禁(见函数注释)
        let t0 = std::time::Instant::now();
        last = evaluate_compound_flow(&reg, &ev);
        times.push(t0.elapsed());
    }
    times.sort();
    println!(
        "216 格单事件:中位 {:?}(9 次;{:?}..{:?};cut_q = {},detached = {} 格)",
        times[times.len() / 2],
        times[0],
        times[times.len() - 1],
        last.cut_q,
        last.detached.len()
    );
}
