"""demos/compound-flow 共用工具(REQ-010 修订 12 单元 G00 demo,§5.8)。

坐标与材料同 demos/impact-flow/common.py(局部坐标,地表 y = 0;超平坦地面 =
y −4 bedrock、y −3…−1 dirt;stone I 18 / I_c 72、dirt 7 / 28、sand 4 / 16、
bedrock ∞;每格 cuboid 半边长 0.45、密度 1 = 0.729 kg;dt = 1/60)。

§5.8.1 惯性汇点的冲量最大流(D16):
- 事件 = (supply {cell: s_c}, J 矢量, p 作用点)。节点 = 复合体的**全部格**
  (无需区域预算与 BFS 截断);
- 源点 → 接触格 c:容量 s_c;相邻格 a → b:u·d̂ ≥ cos 45° 压向 I_c(a,b),
  否则剪切 I(a,b);**格 i → 汇点:惯性容量 d_i = m_i·max(0, Δv_i·d̂)**;
- 0.001 N·s 整数化(供给向下、容量向上);最大流 C;残量网络源可达 = 最小源侧 S;
- 预筛:s < I_min(= 4)直接跳过。

D4 现行规则(§7.1)对照:接触格均摊 J_c = s/|C|;对接触格 c 的每个粘结 (c,m):
J_c ≥ I(c,m) 且非压向(c→m 与 n_c 点积 < cos 45°)→ 断开;n_c = 该格接触法向
(指向本体内部)之和归一化。

rapier 部分沿用 F00 口径:地形 = 固定体多 primitive 薄片(内缩,半厚 0.075,
外表面在格面上,§5.7.10);复合体 = 一个动态刚体挂每格一个 primitive cuboid
(primitive↔primitive 的 point.impulse 精确可读,demos/_API_NOTES.md)。
"""
import math
import sys
import time
from collections import deque

import numpy as np
import rapier3d as r

AIR, STONE, DIRT, BEDROCK, SAND = 0, 1, 2, 3, 4
DIRS6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
INF_CAP = 1 << 24
HALF_SLAB = 0.075
CELL_HALF = 0.45
GRAVITY = (0.0, -9.81, 0.0)
DT = 1.0 / 60.0
CELL_MASS = 0.729
I_OF = {STONE: 18, DIRT: 7, SAND: 4, BEDROCK: INF_CAP, AIR: INF_CAP}
IC_OF = {STONE: 72, DIRT: 28, SAND: 16, BEDROCK: INF_CAP, AIR: INF_CAP}
I_MIN = min(min(I_OF[m], IC_OF[m]) for m in (STONE, DIRT, SAND))  # = 4(sand)
IMPACT_MIN_APPROACH_SPEED = 1.0
QUANT = 1000
COS45 = math.cos(math.pi / 4.0)


def q_supply(x):
    return int(math.floor(x * QUANT))


def q_cap(x):
    if x >= INF_CAP:
        return INF_CAP
    return int(math.ceil(x * QUANT))


def bond_impact(ma, mb):
    ia = I_OF.get(ma, I_OF[BEDROCK])
    ib = I_OF.get(mb, I_OF[BEDROCK])
    return min(ia, ib)


def bond_crush(ma, mb):
    ca = IC_OF.get(ma, IC_OF[BEDROCK])
    cb = IC_OF.get(mb, IC_OF[BEDROCK])
    return min(ca, cb)


def cell_center(c):
    return np.array((c[0] + 0.5, c[1] + 0.5, c[2] + 0.5), float)


# ---------------------------------------------------------------- 最大流

class FlowGraph:
    """整数容量网络;边成对存。node 0 = 源,1 = 汇。(同 demos/impact-flow/common.py)"""

    def __init__(self, n):
        self.n = n
        self.to, self.cap = [], []
        self.adj = [[] for _ in range(n)]

    def add_edge(self, u, v, cap):
        eid = len(self.to)
        self.adj[u].append(eid)
        self.to.append(v)
        self.cap.append(cap)
        self.adj[v].append(eid + 1)
        self.to.append(u)
        self.cap.append(0)

    def ek_maxflow(self):
        """Edmonds–Karp。返回 (maxflow, cap 数组副本)。"""
        cap = list(self.cap)
        flow = 0
        while True:
            prev = [-1] * self.n
            prev[0] = 0
            q = deque([0])
            while q and prev[1] == -1:
                u = q.popleft()
                for eid in self.adj[u]:
                    if cap[eid] > 0 and prev[self.to[eid]] == -1:
                        prev[self.to[eid]] = eid
                        q.append(self.to[eid])
            if prev[1] == -1:
                break
            path = []
            v = 1
            while v != 0:
                eid = prev[v]
                path.append(eid)
                v = self.to[eid ^ 1]
            f = min(cap[eid] for eid in path)
            for eid in path:
                cap[eid] -= f
                cap[eid ^ 1] += f
            flow += f
        return flow, cap

    def dinic_maxflow(self):
        """Dinic(层次图 + 阻塞流,迭代加深 DFS)。"""
        cap = list(self.cap)
        flow = 0
        while True:
            level = [-1] * self.n
            level[0] = 0
            q = deque([0])
            while q:
                u = q.popleft()
                for eid in self.adj[u]:
                    v = self.to[eid]
                    if cap[eid] > 0 and level[v] == -1:
                        level[v] = level[u] + 1
                        q.append(v)
            if level[1] == -1:
                break
            it = [0] * self.n

            def dfs(u, f):
                if u == 1:
                    return f
                while it[u] < len(self.adj[u]):
                    eid = self.adj[u][it[u]]
                    v = self.to[eid]
                    if cap[eid] > 0 and level[v] == level[u] + 1:
                        d = dfs(v, min(f, cap[eid]))
                        if d > 0:
                            cap[eid] -= d
                            cap[eid ^ 1] += d
                            return d
                    it[u] += 1
                return 0

            sys.setrecursionlimit(max(10000, self.n * 2))
            while True:
                f = dfs(0, INF_CAP)
                if f == 0:
                    break
                flow += f
        return flow, cap

    def reachable_from_src(self, cap):
        seen = {0}
        q = deque([0])
        while q:
            u = q.popleft()
            for eid in self.adj[u]:
                v = self.to[eid]
                if cap[eid] > 0 and v not in seen:
                    seen.add(v)
                    q.append(v)
        seen.discard(0)
        return seen


# ---------------------------------------------------------------- §5.8.1 模型

def quat_to_mat(rot):
    """rapier Rotation3(有 .quaternion = (x,y,z,w))→ 3×3 旋转矩阵。"""
    xq, yq, zq, wq = rot.quaternion
    return np.array([
        [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - wq * zq),
         2 * (xq * zq + wq * yq)],
        [2 * (xq * yq + wq * zq), 1 - 2 * (xq * xq + zq * zq),
         2 * (yq * zq - wq * xq)],
        [2 * (xq * zq - wq * yq), 2 * (yq * zq + wq * xq),
         1 - 2 * (xq * xq + yq * yq)]])


def body_pose(world, h):
    """(t_b, R_b):刚体平移(np)与旋转矩阵(np)。"""
    b = world.rigid_bodies[h]
    t_b = np.array(b.translation.to_tuple(), float)
    R_b = quat_to_mat(b.rotation)
    return t_b, R_b


def collider_matches_cells(world, h, cells, offsets):
    """断言:刚体 h 的每个 collider 的 translation_wrt_parent 与格→offset 一致
    (G00-2 修正①:不依赖 colliders.handles() 的创建顺序)。返回 cell_of。"""
    by_off = {}
    for c, off in zip(cells, offsets):
        by_off[tuple(float(v) for v in off)] = c
    hs = [ch for ch in world.colliders.handles()
          if world.colliders.get(ch).parent == h]
    if len(hs) != len(cells):
        raise AssertionError(f"collider 数 {len(hs)} ≠ 格数 {len(cells)}")
    cell_of = {}
    for ch in hs:
        tw = tuple(float(v) for v in
                   world.colliders.get(ch).translation_wrt_parent.to_tuple())
        if tw not in by_off:
            raise AssertionError(f"collider offset {tw} 不在任何格的 offsets 中")
        cell_of[ch] = by_off[tw]
    return cell_of


def rigid_response(cells, mats, p, J, pose=None, centers=None):
    """整体刚体响应(§5.8.1):Δω = I_w⁻¹((p−c)×J)、Δv_i = J/M + Δω×(x_i−c)。

    **事件统一在刚体局部系构造**(G00-2 修正①):p 与 J 必须与格心同系。
    centers: {cell: np.ndarray(3)} **局部系格心**——rapier 实测侧传 offsets
    (collider 的 translation_wrt_parent,裁定①);None = cell_center(c)=c+0.5
    (flow.py 纯函数工况,p 也在该口径,自洽)。
    pose: (origin, R) 仅用于把局部系格心映射到世界做展示,不影响求值
    (刚体响应在 R 下不变,局部系求值 = 世界系求值)。
    cells: [(x,y,z),...];mats: {cell: 材质}。
    M、c、I:逐格 cuboid 半边长 0.45 密度 1,平行轴定理,**含惯性积**(§5.7.2);
    单格自身惯量 = m(2h)²/6(立方体绕任一轴,rapier 对照核过)。
    返回 (M, com, dhat, dv_by_cell {cell: np.ndarray(3)}, domega)。
    """
    n = len(cells)
    M = n * CELL_MASS
    if centers is None:
        centers = {c: cell_center(c) for c in cells}
    centers = np.array([np.asarray(centers[c], float) for c in cells])
    com = centers.mean(axis=0)
    J = np.asarray(J, float)
    p = np.asarray(p, float)
    # 完整惯量张量(格心同系;含惯性积)
    I = np.zeros((3, 3))
    h = CELL_HALF
    for ctr in centers:
        d = ctr - com
        # 单格对自身质心的惯量(立方体,绕任一轴 = m(a²+b²)/12 = m(2h)²/6;
        # 写成 /12 会让 Δω 偏大 2 倍 —— rapier apply_impulse_at_point 对照核过)
        s = CELL_MASS * (2 * h) ** 2 / 6.0
        I += np.diag([s, s, s])
        I += CELL_MASS * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    r = p - com
    torque = np.cross(r, J)
    domega = np.linalg.solve(I, torque)
    v_com = J / M
    dv = {}
    for c, ctr in zip(cells, centers):
        dv[c] = v_com + np.cross(domega, ctr - com)
    Jm = float(np.linalg.norm(J))
    dhat = J / Jm if Jm > 1e-12 else np.zeros(3)
    return M, com, dhat, dv, domega


def compound_flow(cells, mats, supply, J, p, algos=("ek", "dinic"), zero_edges=None,
                  centers=None):
    """§5.8.1 完整求值(纯函数;一个事件)。

    cells: 复合体全部格;mats: {cell: 材质};supply: {接触格: s_c};
    J: 冲量矢量;p: 作用点;p/J 与 centers(格心,同 rigid_response)同系
    (rapier 实测侧传 offsets 口径;纯函数工况缺省 c+0.5)。
    zero_edges: {frozenset({a,b}): 强制容量 0} 的边集合(§5.8.2 校验用)。
    返回 dict: s, C(int, 量化), S(set), cut_bonds, dhat, dv, d_i,
    algos_agree, prefiltered, n_nodes, n_edges, t_flows。
    """
    J = np.asarray(J, float)
    s_total = sum(supply.values())
    M, com, dhat, dv, _ = rigid_response(cells, mats, p, J, centers=centers)
    out = {"s": s_total, "dhat": dhat, "dv": dv, "M": M, "com": com,
           "t_flows": {}, "algos_agree": True, "prefiltered": False}
    if s_total < I_MIN:
        out.update(C=None, S=set(), cut_bonds=0, n_nodes=0, n_edges=0,
                   prefiltered=True, d_i={})
        return out
    cs = list(cells)
    idx = {c: i + 2 for i, c in enumerate(cs)}
    cellset = set(cs)
    SRC, SNK = 0, 1
    g = FlowGraph(len(cs) + 2)
    n_edges = 0
    d_i = {}
    for c in cs:
        uc = idx[c]
        mat_c = mats[c]
        # 格 → 汇点:惯性容量 d_i = m_i·max(0, Δv_i·d̂)
        di = CELL_MASS * max(0.0, float(np.dot(dv[c], dhat)))
        d_i[c] = di
        g.add_edge(uc, SNK, q_cap(di))
        n_edges += 1
        sc = supply.get(c, 0.0)
        if sc > 0:
            g.add_edge(SRC, uc, q_supply(sc))
            n_edges += 1
        for d in DIRS6:
            b = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
            if b not in cellset:
                continue
            dot = float(np.dot(np.array(d, float), dhat))
            cap = bond_crush(mat_c, mats[b]) if dot >= COS45 else bond_impact(mat_c, mats[b])
            if zero_edges and frozenset((c, b)) in zero_edges:
                cap = 0.0
            g.add_edge(uc, idx[b], q_cap(cap))
            n_edges += 1
    out["n_nodes"], out["n_edges"] = g.n, n_edges
    flows, Ss = {}, {}
    for algo in algos:
        t0 = time.perf_counter()
        flow, cap = g.ek_maxflow() if algo == "ek" else g.dinic_maxflow()
        out["t_flows"][algo] = time.perf_counter() - t0
        flows[algo] = flow
        reach = g.reachable_from_src(cap)
        Ss[algo] = {cs[i - 2] for i in reach if 2 <= i < len(cs) + 2}
    out["C"] = flows[algos[0]]
    out["S"] = Ss[algos[0]]
    out["algos_agree"] = (all(f == flows[algos[0]] for f in flows.values())
                          and all(Ss[a] == Ss[algos[0]] for a in Ss))
    out["d_i"] = d_i
    out["cut_bonds"] = count_cut_bonds(cells, mats, out["S"])
    return out


def count_cut_bonds(cells, mats, S):
    """S 与 R 之间被切断的粘结数。"""
    Sset = set(S)
    cellset = set(cells)
    n = 0
    for c in Sset:
        for d in DIRS6:
            b = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
            if b in cellset and b not in Sset:
                n += 1
    return n


def cut_capacity(cells, mats, S):
    """断面容量 T = 被割粘结的容量和(§5.8.2;压向/剪拉按断面方向?——断面
    一律按撞击粘结 I 计,D17 的 T_j 是"粘结容量和";demo 里 T_j 用
    min(I_a,I_b) 与割的方向无关,同 §5.7.2 的断面处理)。"""
    Sset = set(S)
    cellset = set(cells)
    t = 0.0
    for c in Sset:
        for d in DIRS6:
            b = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
            if b in cellset and b not in Sset:
                t += bond_impact(mats[c], mats[b])
    return t


def d4_rule(cells, mats, supply, J, contact_pts):
    """D4 现行规则(§7.1)对照:返回断开的粘结集合 {(a, b)}。

    contact_pts: [(cell, impulse, world_pos, normal_toward_body),...](本事件
    计入的接触点;§7.1 的 C 与 n_c 由它算)。J 合计 = Σ点冲量(方向已含在
    normal 里)。
    """
    s_total = sum(supply.values())
    n_c = {}
    for (cell, imp, pos, n) in contact_pts:
        acc = n_c.setdefault(cell, np.zeros(3))
        acc += np.asarray(n, float)
    broken = set()
    cellset = set(cells)
    jc = s_total / len(n_c)
    for c, acc in n_c.items():
        f = acc / float(np.linalg.norm(acc))
        for d in DIRS6:
            m = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
            if m not in cellset:
                continue
            u = np.array(d, float)
            if float(np.dot(u, f)) >= COS45:
                continue  # 压向豁免
            if jc >= bond_impact(mats[c], mats[m]):
                broken.add(tuple(sorted((c, m))))
    return broken


# ---------------------------------------------------------------- 工况形状

def beam_cells(n=6):
    return [(x, 0, 0) for x in range(n)]


def pillar_cells(n=6):
    return [(0, y, 0) for y in range(n)]


def plate_cells():
    return [(x, 0, z) for x in range(3) for z in range(3)]


def lshape_cells():
    return [(0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 2, 0)]


def seven_cells():
    """req010-impact-corner 的 7 字(6 格,修订 11 定稿几何):柱 x=3、楣 y=2。"""
    return [(0, 2, 0), (1, 2, 0), (2, 2, 0), (3, 2, 0), (3, 1, 0), (3, 0, 0)]


def cube_cells():
    return [(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)]


def tbeam_cells():
    """T 形:竖 3 格 + 顶横 3 格(互撞工况用)。"""
    return [(0, 0, 0), (0, 1, 0), (0, 2, 0), (-1, 2, 0), (1, 2, 0), (0, 3, 0)]


# ---------------------------------------------------------------- ASCII

def profile_ascii(S, cells, plane="xy"):
    """S 的剖面 ASCII(# = S 内,. = 本体其他格)。plane = xy(z 固定取 S 的众数 z)
    或 xz(y 同理)。"""
    Sset = set(S)
    allset = set(cells)
    if plane == "xy":
        zs = sorted({c[2] for c in Sset}) or [0]
        z = max(set(zs), key=lambda zz: sum(1 for c in cells if c[2] == zz))
        pts = {(c[0], c[1]) for c in allset if c[2] == z}
        rows = []
        for y in sorted({p[1] for p in pts}, reverse=True):
            xs = [p[0] for p in pts]
            row = "".join("#" if (x, y) in Sset else "." for x in range(min(xs), max(xs) + 1))
            rows.append(f"y={y:+03d} {row}")
        return "\n".join(rows)
    ys = sorted({c[1] for c in Sset}) or [0]
    y = max(set(ys), key=lambda yy: sum(1 for c in cells if c[1] == yy))
    pts = {(c[0], c[2]) for c in allset if c[1] == y}
    rows = []
    for z in sorted({p[1] for p in pts}, reverse=True):
        xs = [p[0] for p in pts]
        row = "".join("#" if (x, z) in Sset else "." for x in range(min(xs), max(xs) + 1))
        rows.append(f"z={z:+03d} {row}")
    return "\n".join(rows)


# ---------------------------------------------------------------- rapier 工具

def flat_ground_vox(nx=41, nz=41, dirt_layers=3, head=30):
    """超平坦体素(局部坐标,地表 y = 0)。"""
    vox = np.zeros((nx, dirt_layers + 2 + head, nz), dtype=np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:dirt_layers + 1, :] = DIRT
    return vox, -nx // 2, -dirt_layers - 1, -nz // 2


def exposed_faces(vox):
    """[(face_key,(ix,iy,iz)), ...];边界外按实心处理(同 F00)。"""
    P = np.pad(vox, 1, constant_values=STONE)
    core = P[1:-1, 1:-1, 1:-1]
    masks = {
        ("x", +1): core & ~P[2:, 1:-1, 1:-1],
        ("x", -1): core & ~P[:-2, 1:-1, 1:-1],
        ("y", +1): core & ~P[1:-1, 2:, 1:-1],
        ("y", -1): core & ~P[1:-1, :-2, 1:-1],
        ("z", +1): core & ~P[1:-1, 1:-1, 2:],
        ("z", -1): core & ~P[1:-1, 1:-1, :-2],
    }
    faces = []
    for key, m in masks.items():
        for ix, iy, iz in zip(*np.nonzero(m)):
            faces.append((key, (int(ix), int(iy), int(iz))))
    return faces


def slab_terrain(world, vox, x0, y0, z0, friction=0.6, restitution=0.0):
    """§5.7.10 内缩口径地形:暴露面薄片(半厚 0.075,外表面在格面上)。"""
    parts = []
    metas = []
    for (axis, sign), (ix, iy, iz) in exposed_faces(vox):
        wx, wy, wz = ix + x0, iy + y0, iz + z0
        off = [0.5, 0.5, 0.5]
        half = [0.5, 0.5, 0.5]
        k = "xyz".index(axis)
        off[k] = 1.0 if sign > 0 else 0.0
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
        raise RuntimeError("collider 数不匹配")
    cell_of = {ch: cell for ch, (_, _, cell) in zip(handles, metas)}
    return h, cell_of


def add_compound(world, cells, origin_pos, linvel=(0.0, 0.0, 0.0),
                 angvel=(0.0, 0.0, 0.0), rot_vec=None, material=STONE):
    """复合体:一个动态刚体 + 每格一个 primitive cuboid。

    返回 (handle, offsets, cell_of_collider)。原点 = origin_pos(格中心)。
    """
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
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
         .translation(tuple(float(x) for x in origin_pos))
         .linvel(tuple(linvel)).angvel(tuple(angvel)))
    if rot_vec:
        b = b.rotation(tuple(rot_vec))
    colls = [r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
             .translation(off).density(1.0).friction(0.6).restitution(0.0)
             for off in offsets]
    h = world.add_body(b, colliders=colls)
    # G00-2 修正①:collider↔格 按 translation_wrt_parent 匹配(不依赖创建序),
    # 并断言每个 collider 的 local 平移与期望 offset 逐位一致
    cell_of = collider_matches_cells(world, h, cells, offsets)
    return h, offsets, cell_of


def add_single(world, pos, vel, half=CELL_HALF, ccd=False):
    """单格实体。ccd=True 开连续碰撞检测(G00-2 修正⑤:高速投射物防隧穿)。"""
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(tuple(pos))
         .linvel(tuple(vel)))
    if ccd:
        b = b.ccd_enabled(True)
    return world.add_body(b, colliders=[r.Collider.cuboid(half, half, half)
                                        .density(1.0).friction(0.6)
                                        .restitution(0.0)])


def sample_body_contacts(world, handle, cell_of, tb, others,
                         min_speed=IMPACT_MIN_APPROACH_SPEED, R_b=None):
    """§5.8.1 事件采样:handle 的接触中按勘误 2 计入的流形(按流形取大)。

    tb: {handle: (linvel, angvel)} 本步前快照;others: {handle: (linvel, angvel)}
    或地形(不在其中且 is_fixed → 速度 0)。
    **返回局部系事件**(G00-2 修正①②):points 元素
    (cell, impulse, p_local, d_local)——p_local 已除以 collider 旋转
    (p_l = R_cᵀ(p_w − t_c)),d_local = R_bᵀ d_w 指向本体内;J(局部系)。
    R_b:刚体旋转矩阵(body_pose 给出);None = 轴对齐。
    接近速度筛在世界系做:v_pt = R_b·(局部速度场),法向 n_w = R_b·(R_cᵀ)⁻¹…,
    即把局部法向 R_c·n_l 转世界再筛;对方速度用快照。
    """
    pts = []
    t_b = np.array(world.rigid_bodies[handle].translation.to_tuple(), float)
    if R_b is None:
        R_b = np.eye(3)
    RbT = R_b.T
    for pair in world.narrow_phase.contact_pairs():
        c1, c2 = pair.collider1, pair.collider2
        p1 = world.colliders.get(c1).parent
        p2 = world.colliders.get(c2).parent
        if c1 in cell_of:
            # 我是侧 1:local_n1 指向对方(侧 2),local_n2 指向我 → d_w = local_n2;
            # 接触点取我这一侧的 local_p1
            my_c, other_h, n_f, p_f = c1, p2, "local_n2", "local_p1"
        elif c2 in cell_of:
            my_c, other_h, n_f, p_f = c2, p1, "local_n1", "local_p2"
        else:
            continue
        # 对方速度(勘误 2):地形/固定 = 0;动态对方必须用本步前快照——
        # 没给快照时不能拿步进后的当前值(撞击后对方已减速,接近速度会被
        # 低估)→ 该流形直接计入(宁多勿漏,与"取大"精神一致)
        if other_h in others:
            ov = np.array(tuple(others[other_h][0]), float)
            oav = np.array(tuple(others[other_h][1]), float)
            have_snap = True
        else:
            is_fixed = False
            try:
                is_fixed = bool(world.rigid_bodies[other_h].is_fixed)
            except (KeyError, IndexError, AttributeError):
                pass
            if is_fixed:
                ov = np.zeros(3)
                oav = np.zeros(3)
                have_snap = True
            else:
                ov = np.zeros(3)
                oav = np.zeros(3)
                have_snap = False
        lv, av = np.array(tuple(tb[handle][0]), float), np.array(tuple(tb[handle][1]), float)
        # 我的 collider 世界位姿(t_c, R_c)与局部系(f = R_cᵀ(·−t_c) 逆映射)
        ct = world.colliders.get(my_c)
        t_c = np.array(ct.translation.to_tuple(), float)
        R_c = quat_to_mat(ct.rotation)
        RcT = R_c.T
        # 对方 collider 旋转(法向 local_n 在对方 collider 局部系;对方若为
        # 固定地形,其 collider 无旋转 → R_o = I)
        oc = world.colliders.get(c2 if c1 in cell_of else c1)
        R_o = quat_to_mat(oc.rotation)
        for mf in pair.manifolds:
            n_l = np.array(tuple(getattr(mf, n_f)), float)  # 对方局部系,指向我
            n_w = R_o @ n_l                                  # → 世界系
            # 勘误 2:质心口径与逐点口径取大;接近速度 = 朝对方分量 = dot(v, −n_w)
            if not have_snap:
                worst = float("inf")     # 无对方快照:不筛,计入
            else:
                cen = float(np.dot(lv - ov, -n_w))
                worst = cen
                for pt in mf.points:
                    p_l = np.array(tuple(getattr(pt, p_f)), float)  # 我的 collider 局部
                    p_w = t_c + R_c @ p_l
                    r_me = p_w - t_b
                    v_pt = (lv + np.cross(av, r_me)) - ov
                    worst = max(worst, float(np.dot(v_pt, -n_w)))
            if worst < min_speed:
                continue
            for pt in mf.points:
                if pt.impulse <= 0:
                    continue
                p_l = np.array(tuple(getattr(pt, p_f)), float)
                # 事件统一局部系:作用点 = 刚体局部系坐标(G00-2 修正①)
                p_body_l = RcT @ (t_c + R_c @ p_l - t_b)
                d_body_l = RbT @ n_w
                pts.append((cell_of[my_c], pt.impulse, tuple(p_body_l),
                            tuple(d_body_l)))
    J = np.zeros(3)
    for (_, imp, _, d) in pts:
        J += imp * np.array(d, float)
    return pts, J


def cell_velocities_local(world, h, offsets):
    """步进后各格在**刚体局部系**的速度(Δv 实测):
    v_l = R_bᵀ v_w,i,v_w,i = v_com + ω × (r_w,i − t_b),r_w,i = t_b + R_b·off。
    返回 {格序号: np.ndarray(3)}(按 offsets 的顺序)。"""
    b = world.rigid_bodies[h]
    t_b = np.array(b.translation.to_tuple(), float)
    R_b = quat_to_mat(b.rotation)
    v_com = np.array(b.linvel.to_tuple(), float)
    w_b = np.array(b.angvel.to_tuple(), float)
    out = []
    for off in offsets:
        r_w = R_b @ np.array(off, float)
        v_w = v_com + np.cross(w_b, r_w)
        out.append(R_b.T @ v_w)
    return out


def snapshot_all(world, handles):
    return {h: (world.rigid_bodies[h].linvel, world.rigid_bodies[h].angvel)
            for h in handles}
