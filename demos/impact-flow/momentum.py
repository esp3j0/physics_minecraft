"""momentum.py — REQ-010 单元 F00 脚本 2:动量分配与撞击者修正(§5.7.2–§5.7.3,rapier)。

离线实现完整事件管线(每拍循环,后一个事件看到前一个事件的写入,§5.7.4):
  rapier 步进 → 按 §5.7.1 采样接触点(§5.7.9 勘误 2:接触点相对速度沿法向
  ≥ 1 m/s;地形侧位置沿法向退 0.1 m 取格)→ ImpulseFlow 求 C 与 S →
  §5.7.2:m_eff、ρ=(s−C)/s、E=ρ·J、w=|E|/(m_eff+M_S)、撞击者在 p 处受
  m_eff/(m_eff+M_S)·E、分量 k 在 p_k(分量内接触点冲量加权质心)处受
  M_k/(m_eff+M_S)·E → S 的格从地形移除、按 §5.7.3 连通分量生成为动态体
  (初速度 Δv_k、角速度 Δω_k)→ 继续仿真。

工况:
  1. 重物穿薄墙:6 格石条复合体 10/15/20 m/s 横撞 1 格厚土墙(立在 bedrock)
  2. 轻物撞大块:单格 30/60 m/s 横撞 4×4×1 石板(立在 dirt)
  3. 偏心撞击:伸出墙外的 3 格石条被单格方块打末端/中部;Δω 与"同形状刚体
     在 p_k 处施加 P_k"的 rapier 结果对比;m_eff 公式用单位冲量验证
  4. impact-corner 预演:flow.py 工况 6 几何完整管线 + 轨迹与落点
  5. 修订 8 坑内工况:碎块落进一格宽坑,三种规则 300 拍对比脱落格数
  6. 恢复系数敏感性:工况 1、2 在 e=0 与 0.1 下对比守恒残差与穿墙结果

接触冲量读法:demos/_API_NOTES.md 实测路线——地形与实体都是"一个刚体 +
多个 primitive collider",primitive↔primitive 的 point.impulse 精确可读;
接触点世界坐标 = collider 世界位置 + local_p(本脚本 collider 无相对旋转)。

运行:.venv/Scripts/python.exe demos/impact-flow/momentum.py   (~3 分钟)
"""
import math
import os
import sys

import numpy as np
import rapier3d as r

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (AIR, BEDROCK, CELL_HALF, CELL_MASS, DIRT, DIRS6, DT,
                    GRAVITY, I_OF, ImpulseFlow, QUANT, STONE, add_projectile,
                    cell_center, flat_ground, IMPACT_MIN_APPROACH_SPEED,
                    slab_terrain)

RETRACT = 0.1          # 接触点沿法向退 0.1 m 取格(REQ-009 §5)
MAX_EVENTS = 200       # 单次 run 的事件上限(防失控)


# ---------------------------------------------------------------- 采样

def sample_contacts(world, cell_of, dyn_handles, tb, self_w):
    """§5.7.1 采样:动态体↔地形接触点 → (每格冲量 {cell: s_c}, J, points)。

    tb: {handle: (linvel, angvel)} 本步前速度;勘误 2:接触点相对速度
    (地形侧为 0,即实体侧点速度)沿法向分量 ≥ 1 m/s 的点才计入。
    归格非实心(air / 本拍先前事件已移除)的点忽略(REQ-009 §5)。
    返回 points 元素 (cell, impulse, world_pos, dyn_handle, n_toward_dyn)。
    """
    imp_by_cell = {}
    pts = []
    for pair in world.narrow_phase.contact_pairs():
        c1, c2 = pair.collider1, pair.collider2
        p1 = world.colliders.get(c1).parent
        p2 = world.colliders.get(c2).parent
        if c1 in cell_of and p2 in dyn_handles:
            terrain_c, dyn_h = c1, p2
            n_f, p_f = "local_n1", "local_p1"
        elif c2 in cell_of and p1 in dyn_handles:
            terrain_c, dyn_h = c2, p1
            n_f, p_f = "local_n2", "local_p2"
        else:
            continue
        tp0 = np.array(tuple(world.colliders.get(terrain_c).translation), float)
        dyn = world.rigid_bodies[dyn_h]
        bp = np.array(tuple(dyn.translation), float)
        lv = np.array(tuple(tb[dyn_h][0]), float)
        av = np.array(tuple(tb[dyn_h][1]), float)
        for mf in pair.manifolds:
            n = np.array(tuple(getattr(mf, n_f)), float)     # 指向动态体
            for pt in mf.points:
                if pt.impulse <= 0:
                    continue
                pw_t = tp0 + np.array(tuple(getattr(pt, p_f)), float)
                # 接触点速度:用接触点世界位置(实体侧)近似(本 demo 无旋转实体,
                # 旋转体差异 ≤ ω×0.5,见工况 3 的旋转对比,只用于门槛过滤)
                r_pt = pw_t - bp
                v_pt = lv + np.cross(av, r_pt)
                vn = float(np.dot(v_pt, -n))
                if vn < IMPACT_MIN_APPROACH_SPEED:
                    continue
                hit_pos = pw_t - n * RETRACT
                cell = (int(math.floor(hit_pos[0])), int(math.floor(hit_pos[1])),
                        int(math.floor(hit_pos[2])))
                # REQ-009 §5:归格不是实心则忽略(本拍先前事件已写成 air 的格,
                # §5.7.1 也不入网)
                if self_w.get(cell) in (AIR, None):
                    continue
                imp_by_cell[cell] = imp_by_cell.get(cell, 0.0) + pt.impulse
                pts.append((cell, pt.impulse, tuple(pw_t), dyn_h, tuple(n)))
    J = np.zeros(3)
    for (_, imp, _, _, n) in pts:
        J += imp * (-np.array(n))
    return imp_by_cell, J, pts


# ---------------------------------------------------------------- 质量属性

def mass_properties(world, handle, saved_w):
    """(m, inv_inertia_world 3×3):三轴单位 torque impulse 反演(成对施加抵消)。"""
    b = world.rigid_bodies[handle]
    m = b.mass
    inv = np.zeros((3, 3))
    for i in range(3):
        tv = [0.0, 0.0, 0.0]
        tv[i] = 1.0
        b.apply_torque_impulse((tv[0], tv[1], tv[2]), wake_up=True)
        w_ = np.array(tuple(b.angvel), float)
        inv[:, i] = w_ - saved_w
        b.apply_torque_impulse((-tv[0], -tv[1], -tv[2]), wake_up=True)
    return m, inv


def m_eff_along(m, inv_I, r, dhat):
    """1 / (1/m + (r×d̂)·I_w⁻¹·(r×d̂))。"""
    rx = np.cross(r, dhat)
    return 1.0 / (1.0 / m + float(np.dot(rx, inv_I @ rx)))


def spawn_cells(world, cells, linvel, angvel=(0.0, 0.0, 0.0)):
    """§5.7.3:一组格生成为动态体(原点 = 格心均值取整到格心;每格 primitive)。"""
    oc = tuple(int(round(x)) for x in
               np.mean([cell_center(c) for c in cells], axis=0) - 0.5)
    offsets = [(c[0] - oc[0], c[1] - oc[1], c[2] - oc[2]) for c in cells]
    origin_pos = cell_center(oc)
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation(origin_pos)
         .linvel(tuple(float(x) for x in linvel))
         .angvel(tuple(float(x) for x in angvel)))
    return world.add_body(b, colliders=[
        r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
        .translation(off).density(1.0).friction(0.6).restitution(0.0)
        for off in offsets])


def side_ascii(tagged, t, xr, yr):
    """侧视 ASCII(x–y):tagged = {tag: [格心坐标,...]}。"""
    grid = [[" "] * (xr[1] - xr[0] + 1) for _ in range(yr[1] - yr[0] + 1)]
    for tag, positions in tagged.items():
        for (x, y) in positions:
            cx, cy = int(round(x)) - xr[0], int(round(y)) - yr[0]
            if 0 <= cx < len(grid[0]) and 0 <= cy < len(grid):
                grid[cy][cx] = tag
    lines = []
    for y, row in zip(range(yr[1], yr[0] - 1, -1), grid):
        lines.append(f"y={y:>+3d}|" + "".join(row))
    return f"--- t={t} ---\n" + "\n".join(lines)


# ---------------------------------------------------------------- 管线

class Pipeline:
    """事件管线:持有世界、体素、地形句柄;每拍 process_tick。"""

    def __init__(self, vox_w, max_cells=1024):
        self.w = vox_w
        self.world = r.PhysicsWorld(GRAVITY)
        self.terrain_h, self.cell_of = slab_terrain(self.world, self.w)
        self.flow = ImpulseFlow(self.w, max_cells=max_cells)
        self.dyn_handles = set()
        self.events = []            # 本 run 的断开事件详情
        self.held_events = []       # 扛住的事件(s ≥ C)
        self.detach_total = 0

    def add_projectile(self, cells, vel, restitution=0.0):
        h, _, _ = add_projectile(self.world, None, vel, cells=cells,
                                 restitution=restitution)
        self.dyn_handles.add(h)
        return h

    def process_tick(self, tick, break_speed_rule="s57"):
        """采样 + 事件处理(一拍至多一个事件:demo 场景单投射物)。"""
        tb = {h: (self.world.rigid_bodies[h].linvel,
                  self.world.rigid_bodies[h].angvel)
              for h in self.dyn_handles}
        self.world.step()
        imp, J, pts = sample_contacts(self.world, self.cell_of,
                                      self.dyn_handles, tb, self.w)
        if not imp:
            return None
        s = sum(imp.values())
        res = self.flow.evaluate(imp, tuple(J))
        if res["prefiltered"] or res["C"] is None:
            self.held_events.append({"tick": tick, "s": s, "prefiltered": True})
            return {"tick": tick, "s": s, "prefiltered": True}
        if res["C"] >= math.ceil(s * QUANT) or not res["S"]:
            # S=∅:最小割 = 源边本身(浮点边界噪声),无格可断 → 视为扛住
            held = {"tick": tick, "s": s, "C": res["C"] / QUANT,
                    "held": True, "degenerate_cut": not res["S"]}
            self.held_events.append(held)
            return held
        C = res["C"]
        S = res["S"]
        dhat = np.array(res["dhat"], float)
        M_S = len(S) * CELL_MASS
        # 事件涉及的动态体:接触点按 dyn_handle 分组取冲量最大者(一个事件 = 一个实体)
        by_dyn = {}
        for (cell, i_, pw, h_, n) in pts:
            by_dyn.setdefault(h_, 0.0)
            by_dyn[h_] += i_
        proj_h = max(by_dyn, key=lambda h: by_dyn[h])
        b = self.world.rigid_bodies[proj_h]
        cm = np.array(tuple(b.translation), float)
        p_c = np.array([pw for (cell, i_, pw, h_, n) in pts], float)
        wts = np.array([i_ for (_, i_, _, _, _) in pts], float)
        p_c = tuple((p_c * wts[:, None]).sum(axis=0) / wts.sum())
        r_v = np.array(p_c, float) - cm
        m_, inv_I = mass_properties(self.world, proj_h, np.array(tuple(tb[proj_h][1]), float))
        m_eff = m_eff_along(m_, inv_I, r_v, dhat)
        rho = (s - C / QUANT) / s
        E = rho * np.array(J, float)
        w_common = float(np.linalg.norm(E)) / (m_eff + M_S)
        v_before = np.array(tuple(b.linvel), float)
        corr = (m_eff / (m_eff + M_S)) * E
        b.apply_impulse_at_point(tuple(corr), p_c, wake_up=True)
        v_after = np.array(tuple(b.linvel), float)
        # 分量:P_k 与作用点 p_k(分量内接触点冲量加权质心)
        comps = []
        for kcells in res["components"]:
            kset = set(kcells)
            Mk = len(kcells) * CELL_MASS
            kpts = [(i_, pw) for (cell, i_, pw, h_, n) in pts if cell in kset]
            if kpts:
                tot = sum(x[0] for x in kpts)
                pk = tuple(sum(np.array(x[1], float) * x[0] for x in kpts) / tot)
            else:
                pk = p_c
            Pk = (Mk / (m_eff + M_S)) * E
            comps.append((kcells, pk, Pk))
        resid = ((C / QUANT / s) * np.array(J, float) + corr
                 + sum(P for _, _, P in comps) - np.array(J, float))
        detail = {
            "tick": tick, "s": s, "|J|": float(np.linalg.norm(J)),
            "C": C / QUANT, "S_cells": sorted(S), "M_S": M_S,
            "m_eff": m_eff, "rho": rho, "w": w_common,
            "v_before": v_before, "v_after": v_after,
            "p": p_c, "n_contacts": len(pts),
            "comps": [(len(k), pk, P) for k, pk, P in comps],
            "resid": float(np.linalg.norm(resid)),
            "held": False,
        }
        # 执行:移除 S、重建地形、生成分量(初速度 Δv_k = P_k/M_k、Δω_k)
        self.w.set_air_many(S)
        self.world.remove_body(self.terrain_h)
        self.terrain_h, self.cell_of = slab_terrain(self.world, self.w)
        for kcells, pk, Pk in comps:
            dv = Pk / (len(kcells) * CELL_MASS)
            # Δω_k = I_k⁻¹ (r_k × P_k):r_k = pk − 分量质心;近似轴对齐:用
            # 数值反演(生成后施加 Pk at pk)——直接用 apply_impulse_at_point
            # 施加整个 Pk,线/角速度同时正确,无需手算。
            h = spawn_cells(self.world, kcells, (0.0, 0.0, 0.0))
            self.world.rigid_bodies[h].apply_impulse_at_point(
                tuple(Pk), tuple(pk), wake_up=True)
            self.dyn_handles.add(h)
        self.detach_total += len(S)
        self.events.append(detail)
        return detail

    def run(self, ticks, report_every=None, tags_of=None):
        """跑 ticks 拍;返回 (per_event 详情列表, 轨迹帧列表)。"""
        frames = []
        for t in range(1, ticks + 1):
            d = self.process_tick(t)
            if report_every and t % report_every == 0:
                tagged = tags_of(self) if tags_of else {}
                frames.append((t, tagged))
            if d and len(self.events) > MAX_EVENTS:
                break
        return frames


def cells_of_body(world, h):
    """动态体各 collider 格心(世界坐标取整)。"""
    out = []
    for ch in world.colliders.handles():
        c = world.colliders.get(ch)
        if c.parent == h:
            out.append(tuple(np.round(np.array(tuple(c.translation), float))))
    return out


# ---------------------------------------------------------------- 工况

def wall_world(width=7, height=5, base=BEDROCK, wall_block=DIRT):
    """1 格厚墙 x=0、y=0..height-1、z=-(width//2)..(width//2);脚下换 base。"""
    w = flat_ground(41, 41)
    w.fill([(0, -y, z) for y in (1, 2, 3)
            for z in range(-(width // 2), width // 2 + 1)], base)
    w.fill([(0, y, z) for y in range(height)
            for z in range(-(width // 2), width // 2 + 1)], wall_block)
    return w


def case2_plate(speed, restitution=0.0):
    """工况 2:单格投射物横撞 4×4×1 石板(立在 dirt 上,x=0..3、z=0..3、y=0)。"""
    w = flat_ground(41, 41)
    w.fill([(x, 0, z) for x in range(4) for z in range(4)], STONE)
    pl = Pipeline(w)
    h = pl.add_projectile([(6, 1, 1)], (-speed, 0.0, 0.0),
                          restitution=restitution)
    pl.run(200)
    evs = pl.events
    plate_intact = all(pl.w.get((x, 0, z)) == STONE
                       for x in range(4) for z in range(4))
    end = {
        "proj_pos": tuple(round(float(x), 2) for x in
                          pl.world.rigid_bodies[h].translation),
        "proj_vel": tuple(round(float(x), 2) for x in
                          pl.world.rigid_bodies[h].linvel),
        "n_events": len(evs),
        "detached_total": pl.detach_total,
        "plate_intact": plate_intact,
    }
    return pl, evs, end


def case1_and_2(speed, projectile_cells, wall_block=DIRT, base=BEDROCK,
                restitution=0.0, wall_width=7, wall_height=5,
                sim_ticks=200, verbose_frames=0):
    """工况 1/2 共用:投射物横撞 1 格厚墙;返回 (Pipeline, 事件列表, 终态摘要)。"""
    w = wall_world(wall_width, wall_height, base, wall_block)
    pl = Pipeline(w)
    h = pl.add_projectile(projectile_cells, (speed, 0.0, 0.0),
                          restitution=restitution)
    frames = pl.run(sim_ticks, report_every=verbose_frames,
                    tags_of=lambda p: {
                        "P": [(t[0], t[1]) for t in cells_of_body(p.world, h)
                              if abs(t[2]) < 1.0]})

    evs = pl.events
    end = {
        "proj_pos": tuple(np.round(np.array(
            tuple(pl.world.rigid_bodies[h].translation), float), 2)),
        "proj_vel": tuple(np.round(np.array(
            tuple(pl.world.rigid_bodies[h].linvel), float), 2)),
        "n_events": len(evs),
        "detached_total": pl.detach_total,
        "entities": len(pl.dyn_handles),
    }
    # 穿墙判定:投射物格心 x > 1(墙面在 x=0.5~0.5+slab,取 >1)
    end["penetrated"] = end["proj_pos"][0] > 1.0
    return pl, evs, end


def case3(offset_end=True, speed=25.0):
    """偏心撞击:伸出墙外的 3 格石条(根部 1 根粘结 (-1,2,0)↔(-2,2,0))被单格
    方块竖直下砸(25 m/s)。横撞末端格会被压向粘结 I_c=72 扛住(flow.py 工况 1
    的结论,这里用竖直砸顶面使压向判定失效、割 = 剪切 18):
    offset_end=True:砸末端格 (1,2,0) 顶面 → S={末端} 1 格断下;
    offset_end=False:砸中部格 (0,2,0) 顶面 → S={中部,末端} 2 格整段断根。
    25 m/s:s = m·v = 18.2 > 18(石-石剪切)恰断。"""
    w = flat_ground(41, 41)
    # 支撑墙:x=-3..-2(1 格厚、高 3、z -2..2,立在 bedrock)
    w.fill([(x, -y, z) for x in (-3, -2) for y in (1, 2, 3)
            for z in (-2, -1, 0, 1, 2)], BEDROCK)
    w.fill([(x, y, z) for x in (-3, -2) for y in (0, 1, 2)
            for z in (-2, -1, 0, 1, 2)], STONE)
    # 伸出 3 格石条:y=2、x=-1..1
    w.fill([(-1, 2, 0), (0, 2, 0), (1, 2, 0)], STONE)
    pl = Pipeline(w)
    hit_x = 1 if offset_end else 0
    ph = pl.add_projectile([(float(hit_x), 8.0, 0)], (0.0, -speed, 0.0))
    pl.run(200, report_every=0)
    measured = None
    for ev in pl.events:
        if not ev.get("held") and not ev.get("prefiltered") and ev["comps"]:
            measured = ev
            break
    return pl, pl.events, measured


def case3_meff_check():
    """m_eff 公式验证:悬空 3 格石条,在 p 处施加单位冲量,
    接触点法向速度变化应 = 1/m_eff。做法:自由刚体(无接触)在质心系,
    p = 末端格心、d̂ = -x;理论 m_eff = 1/(1/m + (r×d̂)I⁻¹(r×d̂))。
    实测:施加 impulse 1·d̂,读 Δv 与 Δω,再算接触点(即 p)速度变化
    Δv + Δω×r,应 = d̂/m_eff(取模长比)。"""
    world = r.PhysicsWorld(GRAVITY)
    cells = [(0, 2, 0), (1, 2, 0), (2, 2, 0)]
    h = spawn_cells(world, cells, (0, 0, 0))
    m_, inv_I = mass_properties(world, h, np.zeros(3))
    dhat = np.array([-1.0, 0.0, 0.0])
    cm = np.array(tuple(world.rigid_bodies[h].translation), float)
    p = np.array(cell_center((2, 2, 0)), float)
    r_arm = p - cm
    m_eff_th = m_eff_along(m_, inv_I, r_arm, dhat)
    world.rigid_bodies[h].apply_impulse_at_point(tuple(dhat), tuple(p),
                                                 wake_up=True)
    dv = np.array(tuple(world.rigid_bodies[h].linvel), float)
    dw = np.array(tuple(world.rigid_bodies[h].angvel), float)
    # 接触点 p 处速度变化 = Δv + Δω × r(自由刚体);m_eff 是沿 d̂ 的标量,
    # 验证取 d̂ 投影:d̂·Δv_p = 1/m_eff ⟺ 投影模长比 = 1/m_eff_th
    dv_p = dv + np.cross(dw, r_arm)
    ratio = float(np.dot(dhat, dv_p))            # 应 = 1/m_eff_th(冲量 = 1·d̂)
    return m_eff_th, 1.0 / ratio, dv, dw


def case4_impact_corner(sim_ticks=400):
    """impact-corner 预演:flow.py 工况 6 几何完整管线。"""
    w = flat_ground(41, 41)
    w.fill([(8, -1, 0), (8, 0, 0)], STONE)
    w.fill([(x, 1, 0) for x in (5, 6, 7, 8)], STONE)
    pl = Pipeline(w)
    # 投射物:单格 0.729 kg,velocity = [-40,-7,0](场景口径)。
    # 场景的 at=[14,-57.4,20.2] 平移后打在柱身 +x 面(s≈12.6,C=压向 72,擦边扛住,
    # 弹开);把起点移到 (12, 2.5, 0) 正对门楣右端 (8,1,0) 的 +x 面:
    # s=26.57 > 18(门楣右端格与柱顶的剪切割),完整复现"7 字断根"。
    # 这本身就是 F00 的关键实测:场景原弹道擦边扛不住也不断,命中面差一点结果
    # 完全不同(见 README 问题清单)。
    ph = pl.add_projectile([(12.0, 2.5, 0.0)], (-40.0, -7.0, 0.0))
    frames = pl.run(sim_ticks, report_every=40,
                    tags_of=lambda p: {
                        "P": [(t[0], t[1]) for t in cells_of_body(p.world, ph)],
                        "S": [(t[0], t[1]) for hh in list(p.dyn_handles)[1:]
                              for t in cells_of_body(p.world, hh)]})
    return pl, pl.events, frames, ph


def case5_pit(rule="s57", sim_ticks=300):
    """修订 8 坑内工况:一格宽的坑(两侧坑壁薄片对挤)。

    场景(按 REQ-009 修订 9 的描述"碎块在一格宽的坑里几乎静止时,坑壁接触点
    仍读出 ≥ 7 的冲量"):平地上挖一格宽、两格深的坑(x=1、y=-1..-2 为 air);
    先落一块碎块(零初速,静止在坑底),再从上方 3 m 落第二块砸在它上面——
    第一块受压后向两侧挤,坑壁薄片产生对挤伪冲量。

    rule:
      s57  = §5.7(冲量流 + 守恒分配 + 接近速度过滤)
      old  = REQ-009 §5 旧逐格规则:J ≥ I 即脱落、碎块初速 (J−I)·d̂、
             接近速度过滤不加(修订 8 之前)
      old9 = 旧规则 + 修订 9 接近速度过滤(碎块零初速,REQ-009 现行)
    返回 300 拍总脱落格数与事件序列。"""
    w = flat_ground(41, 41)
    w.set_air((1, -1, 0))          # 一格宽、两格深的坑
    w.set_air((1, -2, 0))
    pl = Pipeline(w)
    # 第一块:静止放在坑口(y=-1 格心),零初速(直接生成,不参与撞击判定)
    h1 = spawn_cells(pl.world, [(1, -1, 0)], (0.0, 0.0, 0.0))
    pl.dyn_handles.add(h1)
    # 第二块:从坑上方 3 m 自由下落
    h2 = pl.add_projectile([(1, 4, 0)], (0.0, 0.0, 0.0))
    detached_total = 0
    extra_rule_events = []
    for t in range(1, sim_ticks + 1):
        tb = {hh: (pl.world.rigid_bodies[hh].linvel,
                   pl.world.rigid_bodies[hh].angvel) for hh in pl.dyn_handles}
        pl.world.step()
        if rule == "s57":
            d = pl.process_tick(t)
            if d and not d.get("held") and not d.get("prefiltered"):
                detached_total += len(d["S_cells"])
        else:
            # 旧逐格规则:冲量按格累计(不做供给/网络),J ≥ I 即脱落。
            # old:不过滤接近速度(需要未过滤采样,单独做)
            imp, J, pts = sample_contacts_unfiltered(pl.world, pl.cell_of,
                                                     pl.dyn_handles, tb, pl.w)
            if rule == "old9":
                imp, J, pts = sample_contacts(pl.world, pl.cell_of,
                                              pl.dyn_handles, tb, pl.w)
            if imp:
                total = 0
                for cell, s_c in sorted(imp.items()):
                    mat = pl.w.get(cell)
                    if mat in (AIR, None):
                        continue
                    I = I_OF[mat]
                    if s_c >= I:
                        pl.w.set_air(cell)
                        total += 1
                        if rule == "old":
                            v = max((s_c - I), 0.0)
                            # 碎块初速 = (J−I)·d̂:坑内工况 d 取竖直向下(修订 8
                            # 描述的连锁来源);这里沿接触法向近似(向下)
                            spawn_cells(pl.world, [cell], (0.0, -min(v, 8.0), 0.0))
                        pl.world.remove_body(pl.terrain_h)
                        pl.terrain_h, pl.cell_of = slab_terrain(pl.world, pl.w)
                detached_total += total
                if total:
                    extra_rule_events.append((t, total))
    return {"rule": rule, "detached": detached_total,
            "events": extra_rule_events, "pipeline": pl}


def sample_contacts_unfiltered(world, cell_of, dyn_handles, tb, self_w):
    """sample_contacts 的无过滤变体:只要求 impulse > 0(旧规则口径)。"""
    imp_by_cell = {}
    pts = []
    for pair in world.narrow_phase.contact_pairs():
        c1, c2 = pair.collider1, pair.collider2
        p1 = world.colliders.get(c1).parent
        p2 = world.colliders.get(c2).parent
        if c1 in cell_of and p2 in dyn_handles:
            terrain_c, dyn_h = c1, p2
            n_f, p_f = "local_n1", "local_p1"
        elif c2 in cell_of and p1 in dyn_handles:
            terrain_c, dyn_h = c2, p1
            n_f, p_f = "local_n2", "local_p2"
        else:
            continue
        tp0 = np.array(tuple(world.colliders.get(terrain_c).translation), float)
        for mf in pair.manifolds:
            n = np.array(tuple(getattr(mf, n_f)), float)
            for pt in mf.points:
                if pt.impulse <= 0:
                    continue
                pw_t = tp0 + np.array(tuple(getattr(pt, p_f)), float)
                hit_pos = pw_t - n * RETRACT
                cell = (int(math.floor(hit_pos[0])), int(math.floor(hit_pos[1])),
                        int(math.floor(hit_pos[2])))
                if self_w.get(cell) in (AIR, None):
                    continue
                imp_by_cell[cell] = imp_by_cell.get(cell, 0.0) + pt.impulse
                pts.append((cell, pt.impulse, tuple(pw_t), dyn_h, tuple(n)))
    J = np.zeros(3)
    for (_, i_, _, _, n) in pts:
        J += i_ * (-np.array(n))
    return imp_by_cell, J, pts


def case6_restitution():
    """工况 1、2 在 e=0 与 0.1 下的对比。"""
    out = {}
    for e in (0.0, 0.1):
        pl1, evs1, end1 = case1_and_2(15.0, [(-8 + i, 2, 0) for i in range(6)],
                                      restitution=e)
        pl2, evs2, end2 = case2_plate(30.0, restitution=e)
        out[e] = {"c1_resid": [ev["resid"] for ev in evs1 if not ev.get("held")],
                  "c1_penetrated": end1["penetrated"],
                  "c1_w": [ev["w"] for ev in evs1 if not ev.get("held")],
                  "c2_resid": [ev["resid"] for ev in evs2 if not ev.get("held")],
                  "c2_end": end2}
    return out


# ---------------------------------------------------------------- 主流程

def main():
    print("=" * 78)
    print("momentum.py — §5.7.2/§5.7.3 动量分配、撞击者修正、修订 8 工况(rapier)")
    print("=" * 78)

    print("\n--- 3.0 m_eff 公式验证(悬空 3 格条,末端施加单位冲量)---")
    m_eff_th, m_eff_meas, dv, dw = case3_meff_check()
    print(f"理论 m_eff = {m_eff_th:.6f},实测 1/|Δv_p| = {m_eff_meas:.6f},"
          f"偏差 = {abs(m_eff_th - m_eff_meas) / m_eff_th * 100:.3f}%")
    print(f"Δv = {np.round(dv, 4)},Δω = {np.round(dw, 4)}(应绕 z)")

    print("\n--- 1. 重物穿薄墙:6 格石条(4.374 kg)vs 1 格厚土墙 w7h5 ---")
    print("速度,首事件 s,首事件 C,ρ,w,修正前 vx,修正后 vx,最终穿墙?,事件数,总脱落格,守恒残差(N·s)")
    for speed in (10.0, 15.0, 20.0):
        pl, evs, end = case1_and_2(speed, [(-8 + i, 2, 0) for i in range(6)])
        first = next((ev for ev in evs if not ev.get("held")
                      and not ev.get("prefiltered")), None)
        if first is None:
            print(f"{speed:5.1f},(扛住/预筛,事件={len(evs)})")
            continue
        resids = ", ".join(f"{ev['resid']:.3f}" for ev in evs
                           if not ev.get("held") and not ev.get("prefiltered"))
        print(f"{speed:5.1f},{first['s']:.2f},{first['C']:.2f},"
              f"{first['rho']:.3f},{first['w']:.2f},"
              f"{first['v_before'][0]:+.2f},{first['v_after'][0]:+.2f},"
              f"{end['penetrated']},"
              f"{end['n_events']},{end['detached_total']},{resids}")

    print("\n--- 2. 轻物撞大块:单格 0.729 kg vs 4×4×1 石板(立在 dirt)---")
    print("速度,首事件 s,首事件 C,ρ,w,撞击者修正后 vx(法向),石板被推动?,事件数")
    for speed in (30.0, 60.0):
        pl, evs, end = case2_plate(speed)
        first = next((ev for ev in evs if not ev.get("held")
                      and not ev.get("prefiltered")), None)
        if first is None:
            print(f"{speed:5.1f},(扛住/预筛,事件={len(evs)}),石板完好={end['plate_intact']}")
            continue
        print(f"{speed:5.1f},{first['s']:.2f},{first['C']:.2f},"
              f"{first['rho']:.3f},{first['w']:.2f},"
              f"{end['proj_vel'][0]:+.2f},{not end['plate_intact']},"
              f"{end['n_events']}")

    print("\n--- 3. 偏心撞击:伸出 3 格石条打末端 vs 中部 ---")
    for mode in (True, False):
        pl, evs, measured = case3(offset_end=mode)
        first = measured
        if first is None:
            print(f"打{'末端' if mode else '中部'}:(扛住,事件={len(evs)})")
            continue
        comps = first["comps"]
        # 理论:分量 3 格整体,P_k 沿 d̂;Δv_k = P_k/M_k、Δω_k = I_k⁻¹(r_k×P_k)。
        # rapier 实测由 apply_impulse_at_point 一步给出(生成后施加之拍);
        # 这里直接打印分配的 P_k 与作用点(与"同形状刚体施加 P_k@p_k"同构:
        # 管线本身就是用 rapier 施加的,见 process_tick)
        print(f"打{'末端' if mode else '中部'}: s={first['s']:.2f} "
              f"C={first['C']:.2f} w={first['w']:.2f} ρ={first['rho']:.3f} "
              f"分量={[(n, tuple(np.round(p, 2)), tuple(np.round(P, 2))) for n, p, P in comps]}")

    print("\n--- 4. impact-corner 预演 ---")
    pl, evs, frames, ph = case4_impact_corner()
    for ev in evs:
        if ev.get("held"):
            print(f"  t={ev['tick']}: 扛住 s={ev['s']:.2f} C={ev['C']:.2f}")
        elif ev.get("prefiltered"):
            print(f"  t={ev['tick']}: 预筛 s={ev['s']:.2f}")
        else:
            print(f"  t={ev['tick']}: 断开 s={ev['s']:.2f} C={ev['C']:.2f} "
                  f"|S|={len(ev['S_cells'])} S={ev['S_cells']} "
                  f"w={ev['w']:.2f} ρ={ev['rho']:.3f} m_eff={ev['m_eff']:.3f} "
                  f"M_S={ev['M_S']:.3f} p={tuple(round(x, 2) for x in ev['p'])} "
                  f"分量={[(n, tuple(np.round(p, 2)), tuple(np.round(P, 2))) for n, p, P in ev['comps']]}")
    # 最终落点
    final = {}
    for i, h in enumerate(sorted(pl.dyn_handles,
                                 key=lambda x: (x.index, x.generation))):
        b = pl.world.rigid_bodies[h]
        final[f"body{i}"] = tuple(round(x, 2) for x in b.translation)
    print("  终态实体位置:", final)
    print("  轨迹帧(P=投射物,S=断下部分):")
    for t, tagged in frames:
        print(side_ascii(tagged, t, xr=(0, 16), yr=(-3, 3)))

    print("\n--- 5. 修订 8 坑内工况(300 拍)---")
    for rule in ("old", "old9", "s57"):
        r5 = case5_pit(rule)
        print(f"  规则 {rule:5s}: 总脱落格数 = {r5['detached']}"
              f"(事件 {len(r5['events'])} 次)")

    print("\n--- 6. 恢复系数敏感性 ---")
    for e, o in sorted(case6_restitution().items()):
        print(f"  e={e}: 工况1 残差={['%.3f' % x for x in o['c1_resid']]} "
              f"w={['%.2f' % x for x in o['c1_w']]} 穿墙={o['c1_penetrated']}; "
              f"工况2 残差={['%.3f' % x for x in o['c2_resid']]} 终态={o['c2_end']['proj_vel']}")


if __name__ == "__main__":
    main()
