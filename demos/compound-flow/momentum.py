"""momentum.py — G00 脚本 2:§5.8 复合体冲量流的 rapier 实测。

离线实现完整管线:rapier 步进 → 按勘误 2 采样复合体接触流形(按流形取大)
→ 事件(J、p、s、接触格均摊供给)→ common.compound_flow 求 C 与 S →
§5.8.2 分配(ρ、E、R 退还、对方修正、S 分量获得;各块速度 = 原刚体速度场
+ 自身所受冲量的刚体响应)→ 原刚体删除、各块按算出的速度生成 → 继续仿真。

工况(任务书 7 项):
  1 刚体响应重建残差:单格 20 m/s 横撞零重力静止 6 格条端格;
  2 一次撞击分几拍:平落 16 m 与端角先着地 8/16 m 的逐拍 s 序列,
    逐拍判定 vs 合并判定;
  3 实测阈值(二分):平落 / 端角 30° / 竖柱 / 横撞端格·第 3 格 / 土板 9 m;
  4 守恒与两条解析校验;
  5 与 §5.7 串联:punch / pillar / corner 几何,复合体自身断不断;
  6 复合体互撞:6 格石条 15 m/s 端对端与 T 形;
  7 代价:216 格单事件耗时,估算 Rust < 0.2 ms。

运行:.venv/Scripts/python.exe demos/compound-flow/momentum.py   (~数分钟)
"""
import math
import os
import sys
import time

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (AIR, BEDROCK, CELL_HALF, CELL_MASS, COS45, DIRS6, DIRT,
                    DT, GRAVITY, HALF_SLAB, I_OF, QUANT, STONE,
                    add_compound, add_single, beam_cells, cell_center,
                    compound_flow, cut_capacity, d4_rule, flat_ground_vox,
                    pillar_cells, plate_cells, rigid_response,
                    sample_body_contacts, seven_cells, slab_terrain,
                    snapshot_all, tbeam_cells)

GRAV = np.array(GRAVITY)


# ================================================================ 管线基元

def make_world(grav=True):
    return r.PhysicsWorld(GRAVITY if grav else (0.0, 0.0, 0.0))


class Compound:
    """一个自由复合体:rapier 句柄 + 格 + 材料 + collider 映射。"""

    def __init__(self, world, cells, origin_pos, linvel=(0, 0, 0),
                 angvel=(0, 0, 0), rot_vec=None, material=STONE):
        self.cells = list(cells)
        self.mats = {c: material for c in self.cells}
        self.material = material
        self.h, self.offsets, self.cell_of = add_compound(
            world, self.cells, origin_pos, linvel, angvel, rot_vec, material)

    def mass(self, world):
        return world.rigid_bodies[self.h].mass


def break_event(world, comp, supply, J, p, other_dyn=None, T_zero=False):
    """对一次事件求值并执行 §5.8 断开+分配。返回 detail(dict)或 None(扛住)。

    other_dyn: {handle: np.ndarray(3) 本步前质心速度}(动态对方,可空);
    T_zero: 校验用——S 与 R 之间的粘结容量置 0 重求 C(T=0 合成工况,
    §5.8.2 解析校验);R 退还仍按 T_j/ΣT_j 拆 E(总和恒 = −E,守恒式闭合)。
    """
    zero_edges = None
    res = compound_flow(comp.cells, comp.mats, supply, J, p)
    if res["prefiltered"] or not res["S"]:
        return {"held": True, "prefiltered": res["prefiltered"], "s": sum(supply.values())}
    s = sum(supply.values())
    if res["C"] >= math.ceil(s * QUANT):
        return {"held": True, "s": s, "C": res["C"] / QUANT}
    S = res["S"]
    if T_zero:
        zero_edges = {frozenset((c, b)) for c in S for b in _neighbors_in(comp.cells, c)
                      if b not in S}
        res = compound_flow(comp.cells, comp.mats, supply, J, p,
                            zero_edges=zero_edges)
    R = set(comp.cells) - S
    body = world.rigid_bodies[comp.h]
    v_b = np.array(tuple(body.linvel), float)
    w_b = np.array(tuple(body.angvel), float)
    M, com, dhat, dv, _ = rigid_response(comp.cells, comp.mats, p, J)
    C = res["C"] / QUANT
    rho = (s - C) / s
    E = rho * np.array(J, float)
    # 断面容量 T(S 一侧被割的粘结);R 按 B 形状连通性分分量 R_j
    T = cut_capacity(comp.cells, comp.mats, S)
    R_comps = components_of(sorted(R), comp.mats)
    S_comps = components_of(sorted(S), comp.mats)
    M_S = len(S) * CELL_MASS
    # 各块速度:先取 B 步进后刚体速度场,再加自身所受冲量的响应
    def vel_field(cells_k):
        ctr = np.mean([cell_center(c) for c in cells_k], axis=0)
        return v_b + np.cross(w_b, ctr - com)

    # 1) R 退还:R_j 在自己的断面质心 q_j 受 −(T_j/ΣT)·E(总和恒 = −E)
    r_items = []
    if R_comps:
        q_list = []
        for comp_r in R_comps:
            t_j = 0.0
            q_acc = np.zeros(3)
            q_w = 0.0
            rset = set(comp_r)
            for c in comp_r:
                for d in DIRS6:
                    b2 = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                    if b2 in set(S):
                        cap = bond_I(comp.mats[c], comp.mats[b2])
                        t_j += cap
                        q_acc += 0.5 * (cell_center(c) + cell_center(b2)) * cap
                        q_w += cap
            q_j = q_acc / q_w if q_w > 0 else np.mean(
                [cell_center(c) for c in comp_r], axis=0)
            q_list.append((comp_r, t_j, q_j))
        tot_t = sum(t for _, t, _ in q_list)
        for comp_r, t_j, q_j in q_list:
            share = (t_j / tot_t) if tot_t > 1e-12 else (len(comp_r) / len(R))
            P_r = -share * E
            r_items.append((comp_r, q_j, P_r))
    # 2) 对方修正(对方 = 动态实体;地形份额无修正)
    corr_total = np.zeros(3)
    corr_each = {}
    s_other = {}
    if other_dyn:
        # 按"对方"的接触冲量占 J 的比例拆账:本 demo 单对方事件,份额 = 1
        for oh, ov in other_dyn.items():
            s_other[oh] = E.copy()
    for oh, E_o in s_other.items():
        ob = world.rigid_bodies[oh]
        m_o = ob.mass
        # m_eff,o:对方在本事件作用点沿 d̂ 的等效质量(单格对方 r=0 → m)
        m_eff_o = m_o
        corr = (m_eff_o / (m_eff_o + M_S)) * E_o
        corr_each[oh] = corr
        corr_total = corr_total + corr
    # 3) S 获得:S 应得的部分 = E − Σ(对方修正);各分量按 M_k/M_S 拆
    s_items = []
    M_S_parts = 0.0
    for comp_s in S_comps:
        M_S_parts += len(comp_s) * CELL_MASS
    E_S = E - corr_total
    for comp_s in S_comps:
        kset = set(comp_s)
        kpts = [(imp, pw) for (c, imp, pw, n) in EVT_PTS if c in kset]
        if kpts:
            tot = sum(x[0] for x in kpts)
            p_k = sum(np.array(x[1], float) * x[0] for x in kpts) / tot
        else:
            p_k = np.mean([cell_center(c) for c in comp_s], axis=0)
        M_k = len(comp_s) * CELL_MASS
        P_k = (M_k / M_S_parts) * E_S
        s_items.append((comp_s, p_k, P_k))
    # 守恒残差:Σ_j(R_j 退还) + Σ_o(对方修正) + Σ_k(S_k 获得) = 0(§5.8.2)
    resid = (sum(P for _, _, P in r_items) + corr_total
             + sum(P for _, _, P in s_items))
    detail = {
        "held": False, "s": s, "C": C, "rho": rho, "|E|": float(np.linalg.norm(E)),
        "S": sorted(S), "R_n": len(R), "T": T,
        "S_comps": [len(k) for k in S_comps], "R_comps": [len(k) for k in R_comps],
        "resid": float(np.linalg.norm(resid)),
        "v_b": v_b, "w_b": w_b, "com": com,
        "r_items": [(k, q, P) for k, q, P in r_items],
        "s_items": [(k, p_k, P) for k, p_k, P in s_items],
        "corr_each": corr_each,
    }
    return detail


EVT_PTS = []   # 当前事件的接触点(模块级传参简化;demo 单线程)


def bond_I(ma, mb):
    from common import I_OF
    return min(I_OF.get(ma, I_OF[BEDROCK]), I_OF.get(mb, I_OF[BEDROCK]))


def _neighbors_in(cells, c):
    cs = set(cells)
    out = []
    for d in DIRS6:
        b = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
        if b in cs:
            out.append(b)
    return out


def components_of(cells_sorted, mats):
    """按共面相邻连通性分分量(§5.8.2:按 B 的形状连通性;粘结 I>0 同材料恒真)。"""
    cs = set(cells_sorted)
    seen, comps = set(), []
    for c in sorted(cs):
        if c in seen:
            continue
        comp, q = {c}, [c]
        seen.add(c)
        while q:
            a = q.pop()
            for d in DIRS6:
                n = (a[0] + d[0], a[1] + d[1], a[2] + d[2])
                if n in cs and n not in seen:
                    seen.add(n)
                    comp.add(n)
                    q.append(n)
        comps.append(sorted(comp))
    return comps


def spawn_piece(world, cells_k, linvel, angvel, material=STONE):
    """一组格作为新复合体/单格生成,原点 = 质心所在格。"""
    ctr = np.mean([cell_center(c) for c in cells_k], axis=0)
    oc = tuple(int(math.floor(x)) for x in ctr)
    origin_pos = cell_center(oc)
    offsets = [(c[0] - oc[0], c[1] - oc[1], c[2] - oc[2]) for c in cells_k]
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation(origin_pos)
         .linvel(tuple(float(x) for x in linvel))
         .angvel(tuple(float(x) for x in angvel)))
    h = world.add_body(b, colliders=[
        r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
        .translation(off).density(1.0).friction(0.6).restitution(0.0)
        for off in offsets])
    handles = [ch for ch in world.colliders.handles()
               if world.colliders.get(ch).parent == h]
    cell_of = {ch: c for ch, c in zip(handles, cells_k)}
    return h, cell_of


def respawn_from_event(world, comp, detail, vel_field):
    """删除原刚体,按 detail 的 r_items / s_items 生成 R 与 S 的各块。

    每块速度 = vel_field(块格)(B 步进后刚体速度场)+ 块在自身作用点受 P 的
    刚体响应(apply_impulse_at_point 直接给出线+角,见 _API_NOTES)。
    返回新句柄列表。
    """
    world.remove_body(comp.h)
    new_handles = []
    for cells_k, q, P in detail["r_items"] + detail["s_items"]:
        v0 = vel_field(cells_k)
        M_k = len(cells_k) * CELL_MASS
        w0 = detail["w_b"] if len(cells_k) > 1 else (0.0, 0.0, 0.0)
        h, cell_of = spawn_piece(world, cells_k, v0, w0)
        if float(np.linalg.norm(P)) > 1e-12:
            world.rigid_bodies[h].apply_impulse_at_point(
                tuple(float(x) for x in P), tuple(float(x) for x in q),
                wake_up=True)
        new_handles.append((h, cells_k, P, q))
    return new_handles


# ================================================================ 工况 1

def case1_rigid_response():
    print("=" * 72)
    print("1 刚体响应重建:单格 20 m/s 横撞零重力静止 6 格条端格")
    print("=" * 72)
    world = make_world(grav=False)
    cells = beam_cells()
    comp = Compound(world, cells, cell_center((0, 0, 0)))
    # 投射物:从 −x 打端格,20 m/s(沿 +x)
    start = tuple(np.array(cell_center((0, 0, 0))) + np.array((-3.0, 0, 0)))
    proj = add_single(world, start, (20.0, 0.0, 0.0))
    # 步进到接触;撞击拍:读 J、p,比较步进前后各格速度
    v_pre = {comp.h: (world.rigid_bodies[comp.h].linvel,
                      world.rigid_bodies[comp.h].angvel)}
    dv_meas = None
    for t in range(1, 30):
        world.step()
        pts, J = sample_body_contacts(world, comp.h, comp.cell_of, v_pre, {})
        if pts:
            body = world.rigid_bodies[comp.h]
            origin = np.array(tuple(body.translation), float)
            w_a = np.array(tuple(body.angvel), float)
            v_com_a = np.array(tuple(body.linvel), float)
            v_after = {c: v_com_a + np.cross(w_a, origin + np.array(off, float) - origin)
                       for c, off in zip(cells, comp.offsets)}
            tot = sum(imp for (_, imp, _, _) in pts)
            pw = sum(np.array(x[2], float) * x[1] for x in pts) / tot
            M, com, dhat, dv_pred, _ = rigid_response(cells, comp.mats, pw, J)
            # 撞击前各格速度全 0(静止),Δv_meas = v_after
            errs = [float(np.linalg.norm(v_after[c] - dv_pred[c])) for c in cells]
            scale = float(np.linalg.norm(J)) / (len(cells) * CELL_MASS)
            dv_meas = (J, tot, errs, scale, t, pw)
            break
        v_pre = {comp.h: (world.rigid_bodies[comp.h].linvel,
                          world.rigid_bodies[comp.h].angvel)}
    if dv_meas is None:
        print("  [未捕获撞击拍]")
        return
    J, tot, errs, scale, t, pw = dv_meas
    print(f"  撞击拍 t={t}:|J| = {float(np.linalg.norm(J)):.3f} N·s"
          f"(直读合计 {tot:.3f}),p = {np.round(pw, 3)}")
    print(f"  逐格残差 |Δv_meas − Δv_pred|(m/s):"
          f" {['%.2e' % e for e in errs]}")
    print(f"  参考量级 s/M = {scale:.2f} m/s;最大残差/量级 = "
          f"{max(errs)/scale*100:.2f}%")


# ================================================================ 工况 2

def drop_scenario(cells, rot_vec, height, mat=STONE, max_ticks=400):
    """自由落体到地面;返回逐拍事件序列 [(tick, s, pts, J)]。

    rot_vec: 旋转的缩放轴角向量,或返回该向量的 callable(二分工况用)。
    """
    if callable(rot_vec):
        rot_vec = rot_vec()
    world = make_world()
    vox, x0, y0, z0 = flat_ground_vox()
    slab_terrain(world, vox, x0, y0, z0)
    # 生成高度:最低点离地 = height
    offs = None
    if rot_vec:
        R = rot_matrix(rot_vec)
    else:
        R = np.eye(3)
    lows = [float((R @ np.array((c[0] + 0.5, c[1] + 0.5, c[2] + 0.5), float))[1])
            - CELL_HALF for c in cells]
    comp = Compound(world, cells, (0.5, 0.5, 0.5), rot_vec=rot_vec, material=mat)
    # 需要 re-spawn 到正确高度:直接删了重建到指定高度
    world.remove_body(comp.h)
    origin_y = height + 0.5 - min(float((R @ np.array(o, float))[1])
                                  for o in comp.offsets)
    comp = Compound(world, cells, (0.5, origin_y, 0.5), rot_vec=rot_vec, material=mat)
    events = []
    for t in range(1, max_ticks + 1):
        handles = [comp.h]
        tb = snapshot_all(world, handles)
        world.step()
        if comp.h not in world.rigid_bodies:
            break
        pts, J = sample_body_contacts(world, comp.h, comp.cell_of, tb, {})
        if pts:
            supply, J, pw, s = event_from_pts(pts)
            events.append((t, s, supply, J, tuple(pw)))
        if world.rigid_bodies[comp.h].is_sleeping and t > 60:
            break
    return events


def rot_matrix(rot_vec):
    axis = np.array(rot_vec, float)
    ang = float(np.linalg.norm(axis))
    if ang < 1e-12:
        return np.eye(3)
    axis /= ang
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def event_from_pts(pts):
    """§5.8.1 事件构造:J = Σ j_k·d_k;p = 冲量加权质心;s = |J|;供给均摊 s/|C|。

    pts 元素 (cell, impulse, world_pos, d_toward_body)。返回 (supply, J, p, s)。
    """
    J = np.zeros(3)
    for (_, imp, _, d) in pts:
        J += imp * np.array(d, float)
    s = float(np.linalg.norm(J))
    if s <= 1e-12:
        return {}, J, np.zeros(3), 0.0
    pw = sum(np.array(x[2], float) * x[1] for x in pts) / sum(x[1] for x in pts)
    cells = {c for (c, _, _, _) in pts}
    supply = {c: s / len(cells) for c in cells}
    return supply, J, pw, s


def contact_supply(pts):
    cells = {c for (c, _, _, _) in pts}
    s = sum(imp for (_, imp, _, _) in pts)
    return {c: s / len(cells) for c in cells}


def case2_multi_tick():
    print("=" * 72)
    print("2 一次撞击分几拍:逐拍 s 序列与逐拍 vs 合并判定")
    print("=" * 72)
    for name, cells, rot, h, mat in (
            ("平落16m石", beam_cells(), None, 16.0, STONE),
            ("端角8m石", beam_cells(), (0.0, 0.0, math.radians(30.0)), 8.0, STONE),
            ("端角16m石", beam_cells(), (0.0, 0.0, math.radians(30.0)), 16.0, STONE)):
        ev = drop_scenario(cells, rot, h, mat)
        seq = [(t, round(s, 3)) for (t, s, _, _, _) in ev if s > 0.01]
        print(f"\n  [{name}] 撞击拍序列(tick, s):{seq[:8]}"
              f"{' …' if len(seq) > 8 else ''}")
        # 逐拍判定:每拍独立过 compound_flow;合并判定:撞击窗口内 s 累加后一次
        verdicts = []
        first_break_diag = None
        for (t, s, supply, J, p) in ev:
            if s < 0.01:
                continue
            res = compound_flow(list(cells), {c: mat for c in cells},
                                supply, np.array(J, float), np.array(p, float))
            br = (not res["prefiltered"]) and bool(res["S"]) and \
                res["C"] < math.ceil(sum(supply.values()) * QUANT)
            verdicts.append((t, s, br, res["C"]))
            if br and first_break_diag is None:
                first_break_diag = (
                    t, np.array(J, float), np.array(p, float), res)
        if first_break_diag is not None:
            t, J, p, res = first_break_diag
            M_r, com, dhat, dv, domega = rigid_response(
                list(cells), {c: mat for c in cells}, p, J)
            di = {c: round(res["d_i"][c], 2) for c in sorted(res["d_i"])}
            print(f"  首个断拍 t={t} 诊断:p={np.round(p, 3)}"
                  f" com={np.round(com, 3)} Δω={np.round(domega, 3)}")
            print(f"      d_i={di} Σd_i={sum(res['d_i'].values()):.2f}"
                  f" C={res['C'] / QUANT:.2f} S={sorted(res['S'])}")
        print(f"  逐拍判定:{[(t, round(s,1), '断' if br else '扛') for (t,s,br,_) in verdicts][:8]}")
        # 合并:把 s ≥ 0.01 的连续拍(允许中间隔 ≤2 拍)合成一次
        merged, acc = [], None
        for (t, s, br, C) in verdicts:
            if acc is None:
                acc = [t, s]
            elif t - acc[0] <= 2 + len(acc) - 1:
                acc[1] += s
            else:
                merged.append(acc)
                acc = [t, s]
        if acc:
            merged.append(acc)
        print(f"  合并判定(窗口内 s 累加):{[(t, round(v,1)) for (t,v) in merged]}")
        sum_s = sum(s for (_, s, br, _) in verdicts)
        print(f"  合计 s = {sum_s:.1f} N·s(理论 |Δp| = "
              f"{len(cells)*CELL_MASS*math.sqrt(2*9.81*h):.1f} 多拍分摊应更小)")


# ================================================================ 工况 3

def critical_height(cells, rot_fn, mat, lo=0.05, hi=96.0, tol=0.05):
    """二分临界落高:断 = 撞击窗口内任一拍判 S 非空。"""

    def broken(h):
        ev = drop_scenario(cells, rot_fn, h, mat)
        for (t, s, supply, J, p) in ev:
            if s < 0.01:
                continue
            res = compound_flow(list(cells), {c: mat for c in cells},
                                supply, np.array(J, float), np.array(p, float))
            if (not res["prefiltered"]) and res["S"] and \
                    res["C"] < math.ceil(sum(supply.values()) * QUANT):
                return True
        return False

    if not broken(hi):
        return None, hi
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if broken(mid):
            hi = mid
        else:
            lo = mid
    return hi, hi


def critical_speed(cells, mat, hit_i, lo=0.5, hi=200.0, tol=0.25):
    """二分临界横撞速度(零重力,单格投射物打第 hit_i 格)。"""

    def broken(u):
        world = make_world(grav=False)
        comp = Compound(world, cells, cell_center((0, 0, 0)))
        hit = cells[hit_i]
        proj = add_single(world, (cell_center(hit)[0] - 3.0,
                                  cell_center(hit)[1], cell_center(hit)[2]),
                          (u, 0.0, 0.0))
        handles = [comp.h, proj]
        for t in range(1, 40):
            tb = snapshot_all(world, [comp.h])
            world.step()
            pts, J = sample_body_contacts(world, comp.h, comp.cell_of, tb, {})
            if pts:
                supply, J, pw, s_abs = event_from_pts(pts)
                res = compound_flow(list(cells), {c: mat for c in cells},
                                    supply, J, np.array(pw, float))
                if (not res["prefiltered"]) and res["S"] and \
                        res["C"] < math.ceil(sum(supply.values()) * QUANT):
                    return True
                return False
        return False

    if not broken(hi):
        return None
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if broken(mid):
            hi = mid
        else:
            lo = mid
    return hi


def tot_safe(pts):
    return sum(imp for (_, imp, _, _) in pts)


def fmt_C(C):
    """C 为 None(预筛)时打印占位。"""
    return "预筛" if C is None else f"{C / QUANT:.2f}"


def case3_thresholds():
    print("=" * 72)
    print("3 实测阈值(二分,rapier 多拍口径)")
    print("=" * 72)
    rows = []
    # 平落:验证 64 m 内都不断
    for mat, mn in ((STONE, "石"), (DIRT, "土")):
        h, hmax = critical_height(beam_cells(), None, mat, hi=64.0)
        rows.append((f"6格条平落({mn})", "不断 ≤64 m" if h is None else f"{h:.2f} m"))
    # 端角 30°
    for mat, mn in ((STONE, "石"), (DIRT, "土")):
        h, _ = critical_height(beam_cells(), lambda: (0.0, 0.0, math.radians(30.0)),
                               mat, hi=96.0)
        rows.append((f"6格条端角30°({mn})", f"{h:.2f} m" if h else f"≥96 m"))
    # 竖柱
    for mat, mn in ((STONE, "石"), (DIRT, "土")):
        h, _ = critical_height(pillar_cells(), None, mat, hi=96.0)
        rows.append((f"6格柱竖直落地({mn})", f"{h:.2f} m" if h else f"≥96 m"))
    # 横撞(高速投射物穿透 → 接触点跨格、逐点法向正负对消,补一条诊断)
    for mat, mn in ((STONE, "石"), (DIRT, "土")):
        u = critical_speed(beam_cells(), mat, 0)
        rows.append((f"横撞端格({mn})", f"{u:.1f} m/s" if u else f"≥200 m/s"))
    for mat, mn in ((STONE, "石"), (DIRT, "土")):
        u = critical_speed(beam_cells(), mat, 2)
        rows.append((f"横撞第3格({mn})", f"{u:.1f} m/s" if u else f"≥200 m/s"))
    # 诊断:端格阈值处 u 的事件内部量(以石为样本)
    u = critical_speed(beam_cells(), STONE, 0)
    if u:
        world = make_world(grav=False)
        comp = Compound(world, beam_cells(), cell_center((0, 0, 0)))
        proj = add_single(world, (cell_center(beam_cells()[0])[0] - 3.0,
                                  cell_center(beam_cells()[0])[1],
                                  cell_center(beam_cells()[0])[2]), (u, 0.0, 0.0))
        for t in range(1, 40):
            tb = snapshot_all(world, [comp.h])
            world.step()
            pts, J = sample_body_contacts(world, comp.h, comp.cell_of, tb, {})
            if pts:
                supply, Jv, pw, s_abs = event_from_pts(pts)
                res = compound_flow(list(beam_cells()),
                                    {c: STONE for c in beam_cells()},
                                    supply, Jv, np.array(pw, float))
                print(f"\n  [诊断] 端格 u={u:.1f} m/s 首拍:s=|J|={s_abs:.2f}"
                      f" vs Σ|j|={sum(x[1] for x in pts):.2f}"
                      f"(穿透跨格、法向对消);接触格="
                      f"{sorted({c for (c, _, _, _) in pts})};"
                      f" S={sorted(res['S'])}(S 是主体 {len(res['S'])} 格,"
                      f"端格留在 R)")
                break
    # 3×3×1 土板 9 m(req010-shatter 几何)
    ev = drop_scenario(plate_cells(), None, 9.0, DIRT)
    out = "不断"
    for (t, s, supply, J, p) in ev:
        if s < 0.01:
            continue
        res = compound_flow(list(plate_cells()), {c: DIRT for c in plate_cells()},
                            supply, np.array(J, float), np.array(p, float))
        if (not res["prefiltered"]) and res["S"] and \
                res["C"] < math.ceil(sum(supply.values()) * QUANT):
            out = f"断(t={t}, s={s:.1f}, S={len(res['S'])}格)"
            break
    rows.append(("3×3×1 土板平落9m", out))
    print(f"\n{'工况':<24}{'实测':>18}")
    for name, v in rows:
        print(f"{name:<24}{v:>18}")
    return rows


# ================================================================ 工况 4

def case4_conservation():
    print("=" * 72)
    print("4 守恒与两条解析校验(§5.8.2)")
    print("=" * 72)
    # 校验 A:纯平移、断面容量置 0 → 撞击者交出 m·M_S/(m+M_S)·u
    print("\n  [校验 A] 纯平移 + T=0:撞击者交出的冲量 = m·M_S/(m+M_S)·u")
    cells = beam_cells()
    mat = STONE
    m_proj = CELL_MASS
    u = 180.0  # 单步冲量 ≈ 0.7·m·v ≈ 92 > s*(横撞端格石 86.4,flow.py)→ 断
    # 合成工况(离线构造,纯代数):J 沿 +x 打在端格 0 上,s 取 rapier 实测量级
    # (case4a 的 112 N·s);T = 0(断面无粘结)→ S 应恰好 = 端格(最小割 = 端格
    # 自身惯性汇?),S 的求值仍走 compound_flow(把断面粘结容量置 0 = 只留
    # 端格一个格参与:等价于单格本体)。校验 §5.8.2 的分配代数:
    #   修正 = m_eff/(m_eff+M_S)·E,S 得 M_S/(m_eff+M_S)·E,守恒 Σ = E。
    world = make_world(grav=False)
    comp = Compound(world, cells, cell_center((0, 0, 0)))
    start = tuple(np.array(cell_center(cells[0])) + np.array((-1.35, 0, 0)))
    proj = add_single(world, start, (u, 0.0, 0.0))
    handles = [comp.h, proj]
    pts, J = [], None
    for t in range(1, 30):
        tb = snapshot_all(world, handles)
        world.step()
        pts, J = sample_body_contacts(world, comp.h, comp.cell_of, tb, {})
        if pts:
            break
    if pts:
        supply, J, pw, s = event_from_pts(pts)
        # 真实事件跑 §5.8.2 分配(T_zero=True:R 不拿断面冲量)
        detail = break_event(world, comp, supply, J, pw,
                             other_dyn={proj: (np.array((u, 0, 0)), np.zeros(3))},
                             T_zero=True)
        # 解析对照:撞上"断下的 S"的完全非弹性解交出 m·M_S/(m+M_S)·u;rapier
        # 实际按整条刚体交出 s;T=0 时差 E = s − I* 依 §5.8.2 拆账,对方份额
        # corr = m/(m+M_S)·E。校验三条:① s ≈ 撞整条的解析(模型闭合);
        # ② corr = m/(m+M_S)·(s − m·M_S/(m+M_S)·u)(§5.8.2 解析式);
        # ③ 守恒残差。
        M_S_real = (len(detail["S"]) * CELL_MASS) if detail and not detail["held"] else None
        analytic_whole = m_proj * (len(cells) * CELL_MASS) / \
            (m_proj + len(cells) * CELL_MASS) * u
        if detail and not detail["held"]:
            S = set(detail["S"])
            M_S = len(S) * CELL_MASS
            corr = float(np.linalg.norm(detail["corr_each"].get(proj, np.zeros(3))))
            # §5.8.2 解析校验(T=0 合成):撞击者交出 s − corr 应 = m·M_S/(m+M_S)·u
            net = s - corr
            analytic = m_proj * M_S / (m_proj + M_S) * u
            print(f"    撞击拍 t={t}:s=|J|={s:.3f};S = {sorted(S)}(M_S={M_S:.3f});"
                  f"C={detail['C']:.3f} E={detail['|E|']:.3f}")
            print(f"    ① s vs 撞整条解析 m·M_R/(m+M_R)·u = {s:.3f} vs "
                  f"{analytic_whole:.3f}(偏差 {abs(s-analytic_whole)/analytic_whole*100:.2f}%)")
            print(f"    ② 交出净冲量 s − corr = {net:.3f} vs 只撞 S 的解析 "
                  f"m·M_S/(m+M_S)·u = {analytic:.3f}"
                  f"(偏差 {abs(net-analytic)/analytic*100:.2f}%)")
            print(f"    守恒残差 |ΣR+Σcorr+ΣS| = {detail['resid']:.2e} N·s")
        else:
            print(f"    [t={t} 事件未断(s={s:.2f})]")
            EVT_PTS = []
    else:
        print("    [未捕获撞击拍]")
    # 校验 B:对方为地形 → S 恰好被止住、R 只减少 T(用接地落体工况)
    print("\n  [校验 B] 接地落体(对方=地形):S 被止住、R 只被断面拖慢")
    ev = drop_scenario(cells, None, 4.0, STONE)
    for (t, s, supply, J, p) in ev:
        if s < 0.01:
            continue
        res = compound_flow(list(cells), {c: STONE for c in cells},
                            supply, np.array(J, float), np.array(p, float))
        br = (not res["prefiltered"]) and res["S"] and \
            res["C"] < math.ceil(sum(supply.values()) * QUANT)
        print(f"    平落4m 石:首撞击拍 t={t} s={s:.1f} → "
              f"{'断 S=' + str(sorted(res['S'])) if br else '不断(§5.8 预期:平落内部不传力,撞击动量被各格惯性汇全部吸收,S=∅ 即 R 不动)'}")
        break
    # 全工况守恒残差在 momentum 各工况 detail 中打印;此处汇总机制说明
    EVT_PTS = []


# ================================================================ 工况 5

def punch_geometry():
    """req010-impact-punch:6 格石条 20 m/s 横撞 1 格厚土墙(墙脚基岩)。"""
    world = make_world()
    vox, x0, y0, z0 = flat_ground_vox(nx=81, nz=61, head=40)
    # 墙:x=0(局部),y 0..4,z −3..3;dirt;脚下 y=−1 bedrock 一排
    for yy in range(0, 5):
        for zz in range(-3, 4):
            vox[0 - x0, yy - y0, zz - z0] = DIRT
    for zz in range(-3, 4):
        vox[0 - x0, -1 - y0, zz - z0] = BEDROCK
    slab_terrain(world, vox, x0, y0, z0)
    bar = Compound(world, beam_cells(), (-6.5, 0.5, 0.5),
                   linvel=(20.0, 0.0, 0.0))
    return world, vox, x0, y0, z0, bar


def case5_series():
    print("=" * 72)
    print("5 与 §5.7 串联:punch / pillar / corner 几何(复合体自身)")
    print("=" * 72)
    # —— punch:石条打墙。§5.7 处理墙(离线:墙割 4×18=72,20 m/s 单格当量
    # 冲量 ≈ m_eff·u;石条按 D4 断 2+4;本 demo 问:§5.8 下石条自身断不断)
    print("\n  [punch] 6 格石条 20 m/s 横撞土墙(§5.7 先处理墙)")
    world, vox, x0, y0, z0, bar = punch_geometry()
    handles = [bar.h]
    result = None
    for t in range(1, 60):
        tb = snapshot_all(world, handles)
        world.step()
        if bar.h not in world.rigid_bodies:
            break
        pts, J = sample_body_contacts(world, bar.h, bar.cell_of, tb, {})
        if pts:
            supply, J, pw, s = event_from_pts(pts)
            res = compound_flow(list(bar.cells), bar.mats, supply,
                                J, np.array(pw, float))
            br = (not res["prefiltered"]) and res["S"] and \
                res["C"] < math.ceil(sum(supply.values()) * QUANT)
            result = (t, s, res)
            print(f"    t={t} 首个接触事件:s={s:.2f} C={fmt_C(res['C'])} "
                  f"→ {'断 S=' + str(sorted(res['S'])) if br else '石条不断(整根继续)'}")
            break
    if result is None:
        print("    [60 拍内无接触事件]")
    # —— pillar:4 格石柱立在土上,单格 20 m/s 侧撞(§5.7 断柱脚)
    print("\n  [pillar] 4 格石柱(§5.7 下整根从柱脚断);§5.8 同几何处理")
    world = make_world()
    vox, x0, y0, z0 = flat_ground_vox(nx=61, nz=61, head=40)
    for yy in range(-1, 0):
        for zz in range(-5, 6):
            for xx in range(-5, 6):
                vox[xx - x0, yy - y0, zz - z0] = STONE if yy == -1 else DIRT
    slab_terrain(world, vox, x0, y0, z0)
    pc = [(0, 0, 0), (0, 1, 0), (0, 2, 0), (0, 3, 0)]
    col = Compound(world, pc, cell_center((0, 0, 0)))
    proj = add_single(world, (cell_center((0, 2, 0))[0] - 4.0,
                              cell_center((0, 2, 0))[1],
                              cell_center((0, 2, 0))[2] + 0.05), (20.0, 0.0, 0.0))
    handles = [col.h, proj]
    for t in range(1, 40):
        tb = snapshot_all(world, handles)
        world.step()
        pts, J = sample_body_contacts(world, col.h, col.cell_of, tb, {})
        if pts:
            supply, J, pw, s = event_from_pts(pts)
            res = compound_flow(list(pc), col.mats, supply,
                                np.array(J, float), np.array(pw, float))
            br = (not res["prefiltered"]) and res["S"] and \
                res["C"] < math.ceil(sum(supply.values()) * QUANT)
            print(f"    t={t} 撞击事件:s={s:.2f} C={fmt_C(res['C'])} → "
                  f"{'断 S=' + str(sorted(res['S'])) if br else '柱不断'}")
            break
    # —— corner:7 字立在土地上(§5.7 整个 7 字从柱脚断);§5.8 下投影射物 40 m/s
    print("\n  [corner] 7 字(6 格)立在土地上,40 m/s 斜撞拐角")
    world = make_world()
    vox, x0, y0, z0 = flat_ground_vox(nx=61, nz=61, head=40)
    for yy in range(-1, 0):
        for zz in range(-6, 7):
            for xx in range(-6, 7):
                vox[xx - x0, yy - y0, zz - z0] = DIRT
    slab_terrain(world, vox, x0, y0, z0)
    sc = seven_cells()
    seven = Compound(world, sc, cell_center((0, 0, 0)))
    pp = cell_center(sc[3]) + np.array((6.0, 0.6, 0.2))
    proj = add_single(world, tuple(pp), (-40.0, -7.0, 0.0))
    handles = [seven.h, proj]
    for t in range(1, 60):
        tb = snapshot_all(world, handles)
        world.step()
        pts, J = sample_body_contacts(world, seven.h, seven.cell_of, tb, {})
        if pts:
            supply, J, pw, s = event_from_pts(pts)
            res = compound_flow(list(sc), seven.mats, supply,
                                np.array(J, float), np.array(pw, float))
            br = (not res["prefiltered"]) and res["S"] and \
                res["C"] < math.ceil(sum(supply.values()) * QUANT)
            print(f"    t={t} 撞击事件:s={s:.2f} C={fmt_C(res['C'])} → "
                  f"{'断 S=' + str(sorted(res['S'])) if br else '7 字不断'}")
            break


# ================================================================ 工况 6

def case6_compound_vs_compound():
    print("=" * 72)
    print("6 复合体互撞:6 格石条 15 m/s 相向(端对端 / T 形)")
    print("=" * 72)
    for tag, other_cells, other_pos, other_vel in (
            ("端对端", beam_cells(), (9.5, 0.5, 0.5), (-15.0, 0.0, 0.0)),
            ("T形", tbeam_cells(), (10.5, 0.5, 0.5), (-15.0, 0.0, 0.0))):
        world = make_world(grav=False)
        a = Compound(world, beam_cells(), (0.5, 0.5, 0.5), linvel=(15.0, 0, 0))
        b = Compound(world, other_cells, other_pos, linvel=other_vel)
        handles = [a.h, b.h]
        done = False
        for t in range(1, 60):
            tb = snapshot_all(world, handles)
            world.step()
            for first, second, other in ((a, b, b), (b, a, a)):
                if first.h not in world.rigid_bodies or second.h not in world.rigid_bodies:
                    done = True
                    break
                pts, J = sample_body_contacts(world, first.h, first.cell_of, tb,
                                              {second.h: (np.array(tuple(
                                                  world.rigid_bodies[second.h].linvel), float),
                                                  np.array(tuple(
                                                      world.rigid_bodies[second.h].angvel), float))})
                if pts:
                    global EVT_PTS
                    EVT_PTS = pts
                    supply, J, pw, s = event_from_pts(pts)
                    detail = break_event(world, first, supply, J, pw,
                                         other_dyn={second.h: tb[second.h]})
                    if detail["held"]:
                        print(f"  [{tag}] t={t} {('A' if first is a else 'B')}: "
                              f"s={s:.2f} → 扛住")
                    else:
                        print(f"  [{tag}] t={t} {('A' if first is a else 'B')}: "
                              f"s={s:.2f} C={detail['C']:.2f} ρ={detail['rho']:.2f} "
                              f"S={len(detail['S'])}格 R={detail['R_n']}格 "
                              f"|E|={detail['|E|']:.2f} 残差={detail['resid']:.1e}")
                        respawn_from_event(world, first, detail,
                                           lambda cells_k: vel_of_field(
                                               world, first, detail, cells_k))
                        handles = [h for h in handles if h != first.h] + \
                                  [h for (h, _, _, _) in
                                   world_new_handles(world, first)]
                    EVT_PTS = []
                    done = True
                    break
            if done:
                break
        if not done:
            print(f"  [{tag}] 60 拍内未接触")


def vel_of_field(world, comp, detail, cells_k):
    ctr = np.mean([cell_center(c) for c in cells_k], axis=0)
    return detail["v_b"] + np.cross(detail["w_b"], ctr - detail["com"])


def world_new_handles(world, comp):
    return getattr(comp, "_new", [])


# ================================================================ 工况 7

def case7_cost():
    print("=" * 72)
    print("7 计算代价:216 格单事件(compound_split_size=6 上限)")
    print("=" * 72)
    cells = [(x, y, z) for x in range(6) for y in range(6) for z in range(6)]
    mats = {c: STONE for c in cells}
    # 端面 36 格受击,冲量沿 +x 作用于端面质心
    supply = {c: 500.0 / 36 for c in cells if c[0] == 0}
    J = np.array((500.0, 0.0, 0.0))
    p = np.mean([cell_center(c) for c in cells if c[0] == 0], axis=0)
    t0 = time.perf_counter()
    res = compound_flow(cells, mats, supply, J, p)
    dt = time.perf_counter() - t0
    # 单次计时受 GC/分配噪声影响大,取 5 次中位数为准
    times = []
    for _ in range(5):
        t0 = time.perf_counter()
        compound_flow(cells, mats, supply, J, p)
        times.append(time.perf_counter() - t0)
    med = sorted(times)[len(times) // 2]
    print(f"  216 格 1 事件:建网+EK+Dinic = {dt*1000:.2f} ms"
          f"(5 次中位 {med*1000:.2f} ms;C={res['C']/QUANT:.1f},"
          f"S={len(res['S'])}格,agree={res['algos_agree']})")
    print(f"  Rust 按 Python×(1/50~1/100) 估 → {med*1000/50:.3f}~{med*1000/100:.3f} ms"
          f"({'< 0.2 ms 达标' if med*1000/50 < 0.2 else '需核查'} )")


# ================================================================ main

def main():
    case1_rigid_response()
    print()
    case2_multi_tick()
    print()
    case3_thresholds()
    print()
    case4_conservation()
    print()
    case5_series()
    print()
    case6_compound_vs_compound()
    print()
    case7_cost()


if __name__ == "__main__":
    main()
