"""demos/inset-slab 共用工具(REQ-010 单元 F00b:地形薄片内缩 demo,D15)。

两种地形薄片口径(本 demo 的自变量,每个工况两种都跑、并排打印):
- straddle(现状,生产口径):薄片中心在面平面上,向空气一侧伸出 HALF_SLAB = 0.075
  (对应 crates/physics-sim/src/collider_build.rs `face_box`:center[axis] = plane_world);
- inset(D15 拍板方向):外表面恰在面平面上,中心向实心一侧移 HALF_SLAB,尺寸不变
  (对应 center[axis] = plane_world ∓ FACE_HALF_THICKNESS)。

复用 F00 的 demos/impact-flow:ImpulseFlow(§5.7.1 纯函数)与完整事件管线
momentum.Pipeline(§5.7.2–§5.7.4),只把地形构造换成上面的双口径版本。
momentum.Pipeline 在 __init__ 与 process_tick(断格重建地形)里都引用它自己模块级的
slab_terrain 名字,本模块把该名字替换为按 w.slab_mode 分派的包装(缺省 straddle,
与原函数逐位一致);下面的 Pipeline 子类在构造时把模式写进 w.slab_mode。

rapier 绑定用法见 demos/_API_NOTES.md(0.36.0 实测口径;不要 pip install)。
运行:.venv/Scripts/python.exe demos/inset-slab/<script>.py
(GBK 控制台先 set PYTHONIOENCODING=utf-8)
"""
import importlib.util
import math
import os
import sys

import numpy as np
import rapier3d as r

_DIR = os.path.dirname(os.path.abspath(__file__))
_IF_DIR = os.path.abspath(os.path.join(_DIR, "..", "impact-flow"))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# impact-flow 的 common.py 以私有名加载;momentum.py 顶部 `from common import ...`
# 需要命中它,故临时借住 sys.modules["common"] 再还原(本模块自己以 "common"
# 被脚本导入,不能被顶掉)。
if_common = _load("inset_slab_if_common", os.path.join(_IF_DIR, "common.py"))
_saved = sys.modules.get("common")
sys.modules["common"] = if_common
try:
    M = _load("inset_slab_if_momentum", os.path.join(_IF_DIR, "momentum.py"))
finally:
    if _saved is not None:
        sys.modules["common"] = _saved
    else:
        sys.modules.pop("common", None)

# ---- 再导出(F00 常用面)-------------------------------------------------
AIR = if_common.AIR
STONE = if_common.STONE
DIRT = if_common.DIRT
BEDROCK = if_common.BEDROCK
SAND = if_common.SAND
HALF_SLAB = if_common.HALF_SLAB          # 0.075
CELL_HALF = if_common.CELL_HALF          # 0.45
CELL_MASS = if_common.CELL_MASS          # 0.729
DT = if_common.DT                        # 1/60
GRAVITY = if_common.GRAVITY
G = -GRAVITY[1]                          # 9.81
I_OF = if_common.I_OF
IC_OF = if_common.IC_OF
ImpulseFlow = if_common.ImpulseFlow
flat_ground = if_common.flat_ground
cell_center = if_common.cell_center
profile_ascii = if_common.profile_ascii
add_projectile = M.add_projectile        # (world, pos, vel, ...) 单格或 cells
spawn_cells = M.spawn_cells              # 一组格 → 动态体
sample_contacts = M.sample_contacts
sample_contacts_unfiltered = M.sample_contacts_unfiltered
sample_contacts
cells_of_body = M.cells_of_body
side_ascii = M.side_ascii
RETRACT = M.RETRACT                      # 0.1(生产归格采样深度)

MODES = ("straddle", "inset")
MODE_CN = {"straddle": "跨面(现状)", "inset": "内缩(D15)"}


# ---- 地形:暴露面薄片,双口径 --------------------------------------------

def slab_terrain_mode(world, w, mode="straddle", friction=0.6, restitution=0.0):
    """生产口径地形(固定体 + 每暴露面一片 primitive 薄片),mode 二选一。
    返回 (handle, {collider_handle: 格})。collider↔格 映射按创建序对齐
    (demos/_API_NOTES.md F00 实测)。"""
    if mode not in MODES:
        raise ValueError(mode)
    parts = []
    metas = []
    for (axis, sign), (ix, iy, iz) in if_common.exposed_faces(w.vox):
        wx, wy, wz = ix + w.x0, iy + w.y0, iz + w.z0
        off = [0.5, 0.5, 0.5]
        half = [0.5, 0.5, 0.5]
        k = "xyz".index(axis)
        if mode == "straddle":
            off[k] = 1.0 if sign > 0 else 0.0            # 中心在面平面上
        else:                                            # inset
            off[k] = 1.0 - HALF_SLAB if sign > 0 else HALF_SLAB
        half[k] = HALF_SLAB
        metas.append(((wx + off[0], wy + off[1], wz + off[2]),
                      (half[0], half[1], half[2]), (wx, wy, wz)))
    for t, hf, cell in metas:
        parts.append(r.Collider.cuboid(hf[0], hf[1], hf[2])
                     .translation(t).friction(friction).restitution(restitution))
    h = world.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED), colliders=parts)
    handles = [ch for ch in world.colliders.handles()
               if world.colliders.get(ch).parent == h]
    if len(handles) != len(metas):
        raise RuntimeError(f"collider 数不匹配:{len(handles)} vs {len(metas)}")
    cell_by_collider = {}
    for ch, (_, _, cell) in zip(handles, metas):
        cell_by_collider[ch] = cell
    return h, cell_by_collider


def _slab_terrain_dispatch(world, w):
    """替换 momentum.slab_terrain:断格重建地形时按 w.slab_mode 分派。"""
    return slab_terrain_mode(world, w, getattr(w, "slab_mode", "straddle"))


M.slab_terrain = _slab_terrain_dispatch


class Pipeline(M.Pipeline):
    """F00 完整事件管线,地形构造换成双口径版本。"""

    def __init__(self, vox_w, max_cells=1024, mode="straddle"):
        if mode not in MODES:
            raise ValueError(mode)
        vox_w.slab_mode = mode
        super().__init__(vox_w, max_cells=max_cells)
        self.mode = mode


class WatchPipeline(Pipeline):
    """Pipeline + 每拍记录 watch 体质心(撞击瞬间坐标 / 最大前进距离用)。"""

    def __init__(self, vox_w, max_cells=1024, mode="straddle", watch=None):
        super().__init__(vox_w, max_cells=max_cells, mode=mode)
        self.watch = watch
        self.trace = []

    def process_tick(self, tick):
        d = super().process_tick(tick)
        if self.watch is not None:
            cm = np.array(tuple(self.world.rigid_bodies[self.watch].translation),
                          float)
            self.trace.append((tick, np.round(cm, 4)))
        return d


# ---- 生成辅助 ------------------------------------------------------------

def add_compound(world, cells, vel, face_x0, y_cm, z_cm, restitution=0.0):
    """复合体,身体平移自由指定(cells 只定形状;F00 add_projectile 的 compound
    分支只能把原点放在格心,"反推起点"的对准工况需要非格心平移)。
    face_x0 = 包围盒 +x 外表面(前进方向的前缘)世界坐标——杆在墙左侧、向 +x
    运动,前表面 = 最前格的 +x 面;y_cm / z_cm = 包围盒中心 y / z(对称形状即质心)。"""
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    zs = [c[2] for c in cells]

    def nearest(p):
        d = p - 0.5
        f = math.floor(d)
        return int(f) if (d - f) <= 0.5 else int(f) + 1

    oc = (nearest((min(xs) + max(xs) + 1) / 2.0),
          nearest((min(ys) + max(ys) + 1) / 2.0),
          nearest((min(zs) + max(zs) + 1) / 2.0))
    offsets = [(c[0] - oc[0], c[1] - oc[1], c[2] - oc[2]) for c in cells]
    # 前表面 = pos + offset_max + 0.45 → pos = face_x0 − 0.45 − (max−oc)
    pos = (face_x0 - CELL_HALF - (max(xs) - oc[0]), y_cm, z_cm)
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation(pos).linvel(tuple(float(x) for x in vel)))
    return world.add_body(b, colliders=[
        r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
        .translation(off).density(1.0).friction(0.6).restitution(restitution)
        for off in offsets])


def impact_aim(speed, mode, pen=0.05):
    """反推水平撞击起点:第 t_c 拍接触、该拍穿透 pen 米;竖直补偿
    ycomp 使(自由下坠的)投射物在撞击瞬间质心回到格心高度。
    返回 (t_c, face_x0, ycomp)。墙前表面:跨面 −0.075,内缩 0.0。"""
    gap = 0.925 if mode == "straddle" else 1.0      # 起始面 x=-1.0 到墙前表面
    t_c = max(1, math.ceil(gap / (speed * DT) - 1e-9))
    face = (-0.075 if mode == "straddle" else 0.0) - (t_c * speed * DT - pen)
    ycomp = 0.5 * 9.81 * (t_c * DT) ** 2
    return t_c, face, ycomp


# ---- 纯物理(无撞击判定)工具 ---------------------------------------------

def dyn_terrain_impulse_sum(world, cell_of, dyn_handles):
    """本拍动态体↔地形全部接触点法向冲量合计(不过滤;静止挤压也计入)。"""
    tot = 0.0
    for pair in world.narrow_phase.contact_pairs():
        c1, c2 = pair.collider1, pair.collider2
        p1 = world.colliders.get(c1).parent
        p2 = world.colliders.get(c2).parent
        if ((c1 in cell_of and p2 in dyn_handles)
                or (c2 in cell_of and p1 in dyn_handles)):
            for mf in pair.manifolds:
                for pt in mf.points:
                    if pt.impulse > 0:
                        tot += pt.impulse
    return tot


def run_plain(world, handles, cell_of, ticks, track=None):
    """无撞击判定的纯物理步进。返回每拍 (tick, max_speed, imp_sum, track_cm_y)。"""
    trace = []
    for _ in range(ticks):
        world.step()
        sp = 0.0
        for h in handles:
            b = world.rigid_bodies[h]
            v = np.linalg.norm(np.array(tuple(b.linvel), float))
            wv = np.linalg.norm(np.array(tuple(b.angvel), float)) * CELL_HALF
            sp = max(sp, float(v), float(wv))
        imp = dyn_terrain_impulse_sum(world, cell_of, handles)
        cmy = float("nan")
        if track is not None:
            cmy = float(np.array(tuple(world.rigid_bodies[track].translation),
                                 float)[1])
        trace.append((len(trace) + 1, sp, imp, cmy))
    return trace


def rest_tick_of(trace, eps=0.02, calm=30):
    """首个"其后 calm 拍全部 max_speed < eps"的拍;不存在则 None。"""
    n = len(trace)
    for i in range(n - calm + 1):
        if all(trace[j][1] < eps for j in range(i, i + calm)):
            return trace[i][0]
    return None


def final_y(trace, k=20):
    ys = [t[3] for t in trace[-k:] if not math.isnan(t[3])]
    return sum(ys) / len(ys) if ys else float("nan")


# ---- 归格统计用原始接触点 --------------------------------------------------

def sample_points_raw(world, cell_of, dyn_handles):
    """动态体↔地形接触点原始数据(无接近速度过滤)。
    返回 [(truth_cell, retract_cell, impulse, world_pos)]:
    truth = 被触薄片自己的格(cell_of 映射);retract = 生产口径"沿法向退
    RETRACT=0.1 m 取格"(sim.rs:2961、destruction.rs IMPACT_SAMPLE_DEPTH)。"""
    out = []
    for pair in world.narrow_phase.contact_pairs():
        c1, c2 = pair.collider1, pair.collider2
        p1 = world.colliders.get(c1).parent
        p2 = world.colliders.get(c2).parent
        if c1 in cell_of and p2 in dyn_handles:
            terrain_c, n_f, p_f = c1, "local_n1", "local_p1"
        elif c2 in cell_of and p1 in dyn_handles:
            terrain_c, n_f, p_f = c2, "local_n2", "local_p2"
        else:
            continue
        tp0 = np.array(tuple(world.colliders.get(terrain_c).translation), float)
        for mf in pair.manifolds:
            n = np.array(tuple(getattr(mf, n_f)), float)    # 指向动态体
            for pt in mf.points:
                if pt.impulse <= 0:
                    continue
                pw = tp0 + np.array(tuple(getattr(pt, p_f)), float)
                q = pw - n * RETRACT
                cell_r = (int(math.floor(q[0])), int(math.floor(q[1])),
                          int(math.floor(q[2])))
                out.append((cell_of[terrain_c], cell_r, float(pt.impulse),
                            tuple(np.round(pw, 4))))
    return out


# ---- 自检 -----------------------------------------------------------------

def sanity_print():
    """薄片中心自检:地表格 (0,−1,0) 的 +y 面平面在 y=0。"""
    print("selftest 顶面薄片中心 y(面平面 y=0;期望:跨面 +0.000 / 内缩 −0.075):")
    for mode in MODES:
        w = flat_ground(9, 9)
        world = r.PhysicsWorld(GRAVITY)
        h, _ = slab_terrain_mode(world, w, mode)
        coll = [world.colliders.get(ch) for ch in world.colliders.handles()
                if world.colliders.get(ch).parent == h]
        ys = sorted(round(float(np.array(tuple(c.translation), float)[1]), 4)
                    for c in coll)
        print(f"  {mode:9s}: 薄片数={len(coll)},最高薄片中心 y={ys[-1]:+.4f}")
