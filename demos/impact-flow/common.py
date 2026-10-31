"""demos/impact-flow 共用工具(REQ-010 单元 F00 demo,修订 11 §5.7)。

坐标约定同 demos/cohesion/common.py:格 (x,y,z) 覆盖 [x,x+1)^3,中心 (x+0.5,y+0.5,z+0.5),
地表 y = 0;超平坦地面 = y −4 bedrock、y −3…−1 dirt。

撞体素流(§5.7.1)的纯函数部分:
- ImpulseFlow:从被撞格 BFS 区域(层序、层内 BlockPos 升序、上限)建有向容量网络
  (压向 I_c / 剪拉 I / 五类汇点)、0.001 N·s 整数化(供给向下、容量向上)、
  Edmonds–Karp 与 Dinic 各求一遍、残量网络源可达集 = 最小源侧 S;
- 材料表:§3.2 的 B 与 I + §5.7.5 的 I_c 初值(stone/dirt/sand = 4·I,bedrock/未指定 = ∞)。

rapier 部分:沿用 demos/cohesion 的生产口径——地形 = 固定体上多 primitive 薄片
(半厚 0.075),复合体 = 一个动态刚体挂每格一个 primitive cuboid(不用 Collider.compound,
见 demos/_API_NOTES.md;primitive↔primitive 的 point.impulse 精确可读)。
"""
import math
import sys
import time
from collections import deque

import numpy as np
import rapier3d as r

AIR, STONE, DIRT, BEDROCK, SAND = 0, 1, 2, 3, 4
BLOCK_CHAR = {AIR: " ", STONE: "#", DIRT: "=", BEDROCK: "B", SAND: "."}
DIRS6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
INF_CAP = 1 << 24          # 网络里的 ∞ 哨兵(整数)
HALF_SLAB = 0.075
CELL_HALF = 0.45
GRAVITY = (0.0, -9.81, 0.0)
DT = 1.0 / 60.0
CELL_MASS = 0.729          # 密度 1 的 0.9³ cuboid

# §3.2 / §5.7.5 材料表;∞ 用 INF_CAP 表示
I_OF = {STONE: 18, DIRT: 7, SAND: 4, BEDROCK: INF_CAP, AIR: INF_CAP}
IC_OF = {STONE: 72, DIRT: 28, SAND: 16, BEDROCK: INF_CAP, AIR: INF_CAP}
B_OF = {STONE: 8, DIRT: 3, SAND: 0, BEDROCK: INF_CAP, AIR: 0}
DEFAULT_MATERIAL = BEDROCK  # 未指定材料的实心方块:I/B/I_c 全 ∞
I_MIN = min(min(I_OF[m], IC_OF[m]) for m in (STONE, DIRT, SAND))  # = 4(sand)

# §5.7.9 勘误 2:接近速度门槛 1 m/s,按接触点相对速度口径
IMPACT_MIN_APPROACH_SPEED = 1.0

QUANT = 1000               # 0.001 N·s 量化


def q_supply(x):
    """供给向下取整。"""
    return int(math.floor(x * QUANT))


def q_cap(x):
    """容量向上取整;∞ 输入返回哨兵。"""
    if x >= INF_CAP:
        return INF_CAP
    return int(math.ceil(x * QUANT))


def bond_impact(mat_a, mat_b):
    """I(a,b) = min(I_a, I_b);未指定材料的实心方块按 bedrock(∞)。"""
    ia = I_OF.get(mat_a, I_OF[BEDROCK]) if mat_a is not None else I_OF[BEDROCK]
    ib = I_OF.get(mat_b, I_OF[BEDROCK]) if mat_b is not None else I_OF[BEDROCK]
    return min(ia, ib)


def bond_crush(mat_a, mat_b):
    """I_c(a,b) = min(I_c,a, I_c,b)。"""
    ca = IC_OF.get(mat_a, IC_OF[BEDROCK]) if mat_a is not None else IC_OF[BEDROCK]
    cb = IC_OF.get(mat_b, IC_OF[BEDROCK]) if mat_b is not None else IC_OF[BEDROCK]
    return min(ca, cb)


# ---------------------------------------------------------------- 体素世界

class VWorld:
    """有限体素世界;x/z 越界 = 未装载 chunk(get 返回 None);y 越界 = 世界外(空气)。"""

    def __init__(self, vox, x0, y0, z0):
        self.vox = vox
        self.x0, self.y0, self.z0 = x0, y0, z0
        self.nx, self.ny, self.nz = vox.shape

    def get(self, c):
        ix, iy, iz = c[0] - self.x0, c[1] - self.y0, c[2] - self.z0
        if ix < 0 or ix >= self.nx or iz < 0 or iz >= self.nz:
            return None
        if iy < 0 or iy >= self.ny:
            return AIR
        return int(self.vox[ix, iy, iz])

    def set_air(self, c):
        self.vox[c[0] - self.x0, c[1] - self.y0, c[2] - self.z0] = AIR

    def set_air_many(self, cells):
        for c in cells:
            self.set_air(c)

    def fill(self, cells, block=STONE):
        for (x, y, z) in cells:
            self.vox[x - self.x0, y - self.y0, z - self.z0] = block

    def in_core(self, c):
        """本体区内(§5.7.1 汇点 4:本体区外 = 地基)。"""
        ix, iy, iz = c[0] - self.x0, c[1] - self.y0, c[2] - self.z0
        return 0 <= ix < self.nx and 0 <= iy < self.ny and 0 <= iz < self.nz


def flat_ground(nx=61, nz=61, dirt_layers=3, head=24):
    """超平坦:bedrock + dirt 层,上方留空气;地表 y=0。"""
    vox = np.zeros((nx, dirt_layers + 2 + head, nz), dtype=np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:dirt_layers + 1, :] = DIRT
    return VWorld(vox, -nx // 2, -dirt_layers - 1, -nz // 2)


# ---------------------------------------------------------------- §5.7.1 网络

class ImpulseFlow:
    """§5.7.1 冲量最大流。w: VWorld;world_y_min: 汇点 2 的世界底。

    事件:(impulse_by_cell {cell: s_c}, J 矢量)。节点 = 从被撞格 BFS 的实心格,
    层序、层内 BlockPos 升序,至多 max_cells 个;截断处相邻格当汇点(汇 5)。
    """

    def __init__(self, w, world_y_min=None, max_cells=1024):
        self.w = w
        self.world_y_min = world_y_min if world_y_min is not None else w.y0
        self.max_cells = max_cells

    def bfs_region(self, hit_cell):
        """从被撞格出发的实心格 BFS:层序、层内 BlockPos 升序;返回 (cells, truncated)。

        注:本 demo 是单事件离线复现,"本拍已被先前事件写成 air 的格"由调用方先写 air。
        """
        w = self.w
        if w.get(hit_cell) in (AIR, None):
            return [], False
        seen = {hit_cell}
        frontier = [hit_cell]
        region = []
        while frontier and len(region) < self.max_cells:
            frontier.sort()                       # 层内 BlockPos 升序
            nxt = []
            for c in frontier:
                region.append(c)
                if len(region) >= self.max_cells:
                    break
                for d in DIRS6:
                    n = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                    if n in seen:
                        continue
                    seen.add(n)                   # 空气/未装载也标记,不再探
                    if w.get(n) in (AIR, None):
                        continue
                    nxt.append(n)
            frontier = nxt
        truncated = len(frontier) > 0 or len(region) >= self.max_cells
        return region, truncated

    def edge_cap_raw(self, a, ma, b, mb):
        """a→b 容量(未量化):由方向与 d̂ 决定压向/剪拉。在 evaluate 内联使用。"""
        raise NotImplementedError

    def evaluate(self, impulse_by_cell, J, algos=("ek", "dinic")):
        """完整求值。返回 dict:
        s, C, S(set), truncated, visited, cut_bonds, components, region,
        n_nodes, n_edges, t_flows, agree, profile(S 的格数与剖面数据)
        """
        w = self.w
        Jm = math.sqrt(sum(x * x for x in J))
        dhat = tuple(x / Jm for x in J) if Jm > 1e-12 else (0.0, 0.0, 0.0)
        # 被撞格 = 供给最大的格(同值取 BlockPos 升序,确定性)
        hit = max(impulse_by_cell, key=lambda c: (impulse_by_cell[c],
                                                  tuple(-x for x in c)))
        region, truncated = self.bfs_region(hit)
        out = {"s_cells": dict(impulse_by_cell), "J": tuple(J), "dhat": dhat,
               "region": region, "truncated": truncated, "visited": len(region),
               "t_flows": {}, "algos_agree": True, "prefiltered": False}
        s_total = sum(impulse_by_cell.values())
        out["s"] = s_total
        if s_total < I_MIN:
            out.update(C=None, S=set(), cut_bonds=0, components=[],
                       n_nodes=0, n_edges=0, prefiltered=True)
            return out
        if not region:
            out.update(C=0, S=set(), cut_bonds=0, components=[],
                       n_nodes=0, n_edges=0)
            return out
        idx = {c: i + 2 for i, c in enumerate(region)}
        SRC, SNK = 0, 1
        g = FlowGraph(len(region) + 2)
        n_edges = 0
        cos45 = math.cos(math.pi / 4.0)
        for a in region:
            ua = idx[a]
            sa = q_supply(impulse_by_cell.get(a, 0.0))
            if a == hit:
                g.add_edge(SRC, ua, max(sa, 1) if sa > 0 else 0)
                n_edges += 1
            elif sa > 0:
                # 区域内其他格也有供给(多点接触):同样作为源(需求语义不变)
                g.add_edge(SRC, ua, sa)
                n_edges += 1
            ma = w.get(a)
            for d in DIRS6:
                b = (a[0] + d[0], a[1] + d[1], a[2] + d[2])
                u = d
                dot = (u[0] * dhat[0] + u[1] * dhat[1] + u[2] * dhat[2])
                mb = w.get(b)
                cap_raw = (bond_crush(ma, mb) if dot >= cos45
                           else bond_impact(ma, mb))
                if mb is None:
                    cap = q_cap(cap_raw)              # 汇 3:未装载
                    g.add_edge(ua, SNK, cap)
                    n_edges += 1
                    continue
                if mb == AIR:
                    continue
                if not w.in_core(b):
                    cap = q_cap(cap_raw)              # 汇 4:本体区外
                    g.add_edge(ua, SNK, cap)
                    n_edges += 1
                    continue
                if b not in idx:
                    cap = q_cap(cap_raw)              # 汇 5:BFS 截断
                    g.add_edge(ua, SNK, cap)
                    n_edges += 1
                    continue
                g.add_edge(ua, idx[b], q_cap(cap_raw))
                n_edges += 1
            # 汇 1/2:Anchor 材料、WORLD_Y_MIN 向下
            if ma >= INF_CAP:
                g.add_edge(ua, SNK, INF_CAP)
                n_edges += 1
            if a[1] == self.world_y_min:
                g.add_edge(ua, SNK, q_cap(I_OF[ma]))  # 向下方向(近似:材料 I)
                n_edges += 1
        out["n_nodes"], out["n_edges"] = g.n, n_edges
        flows = {}
        Ss = {}
        for algo in algos:
            t0 = time.perf_counter()
            flow, cap = g.ek_maxflow() if algo == "ek" else g.dinic_maxflow()
            out["t_flows"][algo] = time.perf_counter() - t0
            flows[algo] = flow
            reach = g.reachable_from_src(cap)
            Ss[algo] = {region[i - 2] for i in reach if 2 <= i < len(region) + 2}
        out["C"] = flows[algos[0]]
        out["S"] = Ss[algos[0]]
        out["algos_agree"] = (all(f == flows[algos[0]] for f in flows.values())
                              and all(Ss[a] == Ss[algos[0]] for a in Ss))
        out["cut_bonds"] = self.cut_bonds(out["S"])
        out["components"] = bond_components(out["S"], w)
        return out

    def cut_bonds(self, S):
        """S 与其余部分之间的粘结数(I > 0,恰一端在 S)。"""
        w = self.w
        n = 0
        for c in S:
            for d in DIRS6:
                nb = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                if nb not in S and w.get(nb) not in (AIR, None):
                    if bond_impact(w.get(c), w.get(nb)) > 0:
                        n += 1
        return n


def bond_components(cells, w):
    """§5.7.3:按"撞击粘结 I > 0 的共面连接"求连通分量。"""
    cs = set(cells)
    seen, comps = set(), []
    for c in sorted(cs):
        if c in seen:
            continue
        comp, q = {c}, deque([c])
        seen.add(c)
        while q:
            a = q.popleft()
            for d in DIRS6:
                n = (a[0] + d[0], a[1] + d[1], a[2] + d[2])
                if n in cs and n not in seen and bond_impact(w.get(a), w.get(n)) > 0:
                    seen.add(n)
                    comp.add(n)
                    q.append(n)
        comps.append(sorted(comp))
    return comps


# ---------------------------------------------------------------- 最大流

class FlowGraph:
    """整数容量网络;边成对存。node 0 = 源,1 = 汇。"""

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
            # 找瓶颈并增广(路径容量可 > 1)
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


# ---------------------------------------------------------------- ASCII 剖面

def profile_ascii(cells, plane="xz", y=None, z=None):
    """S(或区域)的 x–z / x–y 剖面 ASCII。"""
    cs = set(cells)
    if plane == "xz":
        coords = sorted({(c[0], c[2]) for c in cs})
        if not coords:
            return "(empty)"
        xs = [c[0] for c in coords]
        zs = [c[1] for c in coords]
        rows = []
        for zz in range(max(zs), min(zs) - 1, -1):
            row = "".join("#" if (x, zz) in coords else "." for x in range(min(xs), max(xs) + 1))
            rows.append(f"z={zz:+03d} {row}")
        return "\n".join(rows)
    coords = sorted({(c[0], c[1]) for c in cs if c[2] == z})
    if not coords:
        return "(empty at this z)"
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    rows = []
    for yy in range(max(ys), min(ys) - 1, -1):
        row = "".join("#" if (x, yy) in coords else "." for x in range(min(xs), max(xs) + 1))
        rows.append(f"y={yy:+03d} {row}")
    return "\n".join(rows)


# ---------------------------------------------------------------- rapier:地形与实体

def exposed_faces(vox):
    """[(face_key,(ix,iy,iz)), ...];边界外按实心处理。"""
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


def slab_terrain(world, w, friction=0.6, restitution=0.0):
    """生产口径地形:暴露面薄片(半厚 0.075),固定体 + 每薄片一个 primitive collider。
    返回 (handle, {collider_handle: 格})。"""
    parts = []
    cell_by_collider = {}
    metas = []
    for (axis, sign), (ix, iy, iz) in exposed_faces(w.vox):
        wx, wy, wz = ix + w.x0, iy + w.y0, iz + w.z0
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
    # collider 句柄 → 格:world.colliders.handles() 按创建序产出,与 metas 一一对应
    # (实测:_API_NOTES.md;避免 O(N²) 最近邻匹配)
    handles = [ch for ch in world.colliders.handles()
               if world.colliders.get(ch).parent == h]
    if len(handles) != len(metas):
        raise RuntimeError(f"collider 数不匹配:{len(handles)} vs {len(metas)}")
    for ch, (_, _, cell) in zip(handles, metas):
        cell_by_collider[ch] = cell
    return h, cell_by_collider


def cell_center(c):
    return (c[0] + 0.5, c[1] + 0.5, c[2] + 0.5)


def add_projectile(world, pos, vel, half=CELL_HALF, density=1.0, friction=0.6,
                   restitution=0.0, cells=None):
    """单格投射物(或给定 cells 的复合体)。返回 (handle, offsets, origin_pos)。

    复合体:原点 = 包围盒中心取整到最近格中心(同距取小),offsets = 格 − 原点格。
    """
    if cells is None:
        b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(tuple(pos))
             .linvel(tuple(vel)))
        h = world.add_body(b, colliders=[r.Collider.cuboid(half, half, half)
                                         .density(density).friction(friction)
                                         .restitution(restitution)])
        return h, [(0.0, 0.0, 0.0)], tuple(pos)
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
    origin_pos = cell_center(oc)
    b = (r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(origin_pos)
         .linvel(tuple(vel)))
    h = world.add_body(b, colliders=[r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF)
                                     .translation(off).density(density)
                                     .friction(friction).restitution(restitution)
                                     for off in offsets])
    return h, offsets, origin_pos
