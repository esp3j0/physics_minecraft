"""demos/cohesion 共用工具(REQ-010 任务 00 demo)。

坐标约定同 REQ-009 demo:格 (x,y,z) 覆盖 [x,x+1)^3,中心 (x+0.5,y+0.5,z+0.5),地表 y=0。
体素侧:VWorld + LoadEval(REQ-010 §4 承载判定:接地格缓存、悬挑区、整数容量网络、
最大流 Edmonds–Karp 与 Dinic 各一遍、残量网络源可达集 F)。
rapier 侧:地形 = 固定体上的多个 primitive 薄片 collider(生产口径半厚 0.075;不用
Collider.compound —— 复合 collider 参与的接触对 point.impulse 恒 0,primitive↔primitive
可精确直读,见 demos/_API_NOTES.md);复合体 = 一个动态刚体挂每格一个 primitive cuboid。
"""
import itertools
import math
import time
from collections import deque

import numpy as np
import rapier3d as r

AIR, STONE, DIRT, BEDROCK, SAND = 0, 1, 2, 3, 4
BLOCK_CHAR = {AIR: " ", STONE: "#", DIRT: "=", BEDROCK: "B", SAND: "."}
DIRS6 = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
INF_CAP = 1 << 24          # 容量网络的数值化 ∞
HALF_SLAB = 0.075          # 暴露面薄片半厚(生产口径)
CELL_HALF = 0.45           # 每格 cuboid 半边长
GRAVITY = (0.0, -9.81, 0.0)
DT = 1.0 / 60.0
LOAD_MAX_CELLS = 2048      # REQ-010 §11 默认


# ---------------------------------------------------------------- 体素世界

class VWorld:
    """有限体素世界;x/z 越界 = 未装载 chunk(get 返回 None,视为有地基);
    y 越界 = 世界外(按空气)。"""

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

    def fill(self, cells, block=STONE):
        for (x, y, z) in cells:
            self.vox[x - self.x0, y - self.y0, z - self.z0] = block

    def remove(self, cells):
        self.set_air_many(cells)

    def set_air_many(self, cells):
        for c in cells:
            self.set_air(c)

    def solid_cells(self):
        return {(ix + self.x0, iy + self.y0, iz + self.z0)
                for ix, iy, iz in zip(*np.nonzero(self.vox != AIR))}

    def ground_heights(self):
        """每 (x,z) 柱自底向上连续实心段的顶端 y(段必须触到世界底);空柱 = None。"""
        air = self.vox == AIR
        first = np.argmax(air, axis=1)                 # 沿 y 第一个空气(全实心 = 0)
        any_air = air.any(axis=1)
        has_bottom = ~air[:, 0, :]
        top = np.where(any_air, first - 1, self.ny - 1)
        gh = {}
        for ix in range(self.nx):
            for iz in range(self.nz):
                gh[(ix + self.x0, iz + self.z0)] = (
                    int(top[ix, iz]) + self.y0
                    if has_bottom[ix, iz] and top[ix, iz] >= 0 else None)
        return gh


def flat_ground(nx=61, nz=61, dirt_layers=3, head=24):
    """超平坦:bedrock + dirt_layers 层 dirt,上方留 head 格空气;地表 y=0。"""
    vox = np.zeros((nx, dirt_layers + 2 + head, nz), dtype=np.int8)
    vox[:, 0, :] = BEDROCK
    vox[:, 1:dirt_layers + 1, :] = DIRT
    return VWorld(vox, -nx // 2, -dirt_layers - 1, -nz // 2)


def trigger_of(changed):
    """触发集 = 实心性变化的格 + 6 邻格(REQ-010 §4.4)。"""
    out = set()
    for (x, y, z) in changed:
        out.add((x, y, z))
        for dx, dy, dz in DIRS6:
            out.add((x + dx, y + dy, z + dz))
    return sorted(out)


# ---------------------------------------------------------------- 最大流

class FlowGraph:
    """整数容量网络;边成对存(正边 + 反向残量边)。node 0 = 源,1 = 汇。"""

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
        return eid

    def ek_maxflow(self, cap=None):
        """Edmonds–Karp(BFS 增广)。返回 (maxflow, cap 数组)。"""
        cap = list(self.cap) if cap is None else cap
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
            # 源点边容量 1,每条增广路 1 单位
            v = 1
            while v != 0:
                eid = prev[v]
                cap[eid] -= 1
                cap[eid ^ 1] += 1
                v = self.to[eid ^ 1]
            flow += 1
        return flow, cap

    def dinic_maxflow(self, cap=None):
        """Dinic(层次图 + 阻塞流)。返回 (maxflow, cap 数组)。"""
        cap = list(self.cap) if cap is None else cap
        flow = 0
        level = [0] * self.n
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

            while True:
                f = dfs(0, INF_CAP)
                if f == 0:
                    break
                flow += f
        return flow, cap

    def reachable_from_src(self, cap):
        """残量网络中源点可达的节点集(不含源)。"""
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


# ---------------------------------------------------------------- 承载判定

class LoadEval:
    """REQ-010 §4 承载判定。B: dict block_id -> int(∞ 用 INF_CAP)。

    网络:源→悬挑格 1;悬挑格 a→实心邻格 b(正下 ∞,水平/正上 B(a,b),0 = 无边);
    b 接地或 b 在未装载 chunk → 连汇(后者 ∞);接地格、源、汇不进悬挑区。
    """

    def __init__(self, w, B):
        self.w = w
        self.B = B

    def bond(self, a, b):
        return min(self.B[self.w.get(a)], self.B[self.w.get(b)])

    def overhang_region(self, seeds, max_cells=LOAD_MAX_CELLS, gh=None):
        """从触发格出发,只走非接地实心格的 6 连通区域(BFS,按 seeds 顺序入队)。

        返回 (cells 列表, truncated);访问格数 > max_cells 时提前截断(truncated)。
        """
        gh = gh or self.w.ground_heights()
        region, seen = [], set()
        q = deque()
        for c in seeds:
            if c in seen:
                continue
            b = self.w.get(c)
            if b is None or b == AIR:
                continue
            g = gh.get((c[0], c[2]))
            if g is not None and c[1] <= g:
                continue                      # 接地格
            seen.add(c)
            q.append(c)
        while q:
            c = q.popleft()
            region.append(c)
            if len(region) > max_cells:
                return region, True
            for dx, dy, dz in DIRS6:
                n = (c[0] + dx, c[1] + dy, c[2] + dz)
                if n in seen:
                    continue
                b = self.w.get(n)
                if b is None or b == AIR:
                    continue
                g = gh.get((n[0], n[2]))
                if g is not None and n[1] <= g:
                    continue
                seen.add(n)
                q.append(n)
        return region, False

    def build_graph(self, region, gh):
        """region 顺序决定节点编号(触发顺序无关性测试靠 F 唯一定理兜底)。"""
        w, B = self.w, self.B
        idx = {c: i for i, c in enumerate(region)}
        g = FlowGraph(len(region) + 2)
        SRC, SNK = 0, 1
        n_edges_bond = 0
        for a in region:
            u = idx[a] + 2
            g.add_edge(SRC, u, 1)
            ba = B[w.get(a)]
            for dx, dy, dz in DIRS6:
                b = (a[0] + dx, a[1] + dy, a[2] + dz)
                nb = w.get(b)
                if nb is None:
                    g.add_edge(u, SNK, INF_CAP)          # 未装载 = 有地基
                    continue
                if nb == AIR:
                    continue
                g_ = gh.get((b[0], b[2]))
                grounded = g_ is not None and b[1] <= g_
                if dy == -1:
                    cap = INF_CAP                        # 正下:压,不限量
                else:
                    cap = min(ba, B[nb])                 # 水平/正上:剪与拉
                    if cap <= 0:
                        continue
                if grounded or b not in idx:
                    g.add_edge(u, SNK, cap)              # 接地格为汇
                else:
                    g.add_edge(u, idx[b] + 2, cap)
        return g, idx

    def evaluate(self, seeds, max_cells=LOAD_MAX_CELLS, algos=("ek", "dinic")):
        """完整判定。返回 dict:
        f(F 集,截断时 None=按稳定处理)/ truncated / visited / flow / cut_bonds /
        n_nodes / n_edges / t_region / t_flows(dict algo->s)/ agree(bool)。
        """
        t0 = time.perf_counter()
        gh = self.w.ground_heights()
        region, truncated = self.overhang_region(seeds, max_cells, gh)
        t_region = time.perf_counter() - t0
        out = {"truncated": truncated, "visited": len(region),
               "t_region": t_region, "t_flows": {}}
        if truncated:
            out.update(f=None, flow=None, cut_bonds=0, n_nodes=0, n_edges=0, agree=True)
            return out
        if not region:
            out.update(f=set(), flow=0, cut_bonds=0, n_nodes=0, n_edges=0, agree=True)
            return out
        g, idx = self.build_graph(region, gh)
        out["n_nodes"], out["n_edges"] = g.n, len(g.to) // 2
        out["t_flows"] = {}
        fs = {}
        for algo in algos:
            t0 = time.perf_counter()
            flow, cap = (g.ek_maxflow() if algo == "ek" else g.dinic_maxflow())
            out["t_flows"][algo] = time.perf_counter() - t0
            reach = g.reachable_from_src(cap)
            fs[algo] = {region[i - 2] for i in reach if i >= 2}
            if algo == algos[0]:
                out["flow"] = flow
        out["f"] = fs[algos[0]]
        out["agree"] = all(fs[a] == fs[algos[0]] for a in fs)
        out["cut_bonds"] = self.cut_bonds(out["f"])
        return out

    def cut_bonds(self, f):
        """F 与其余部分之间的粘结数(两侧均实心、共面、B > 0,恰一端在 F)。"""
        fs = set(f)
        n = 0
        for c in fs:
            for dx, dy, dz in DIRS6:
                nb = (c[0] + dx, c[1] + dy, c[2] + dz)
                if nb not in fs and self.w.get(nb) and self.bond(c, nb) > 0:
                    n += 1
        return n


# ---------------------------------------------------------------- 分组与切块

def bond_groups(cells, Beval):
    """按粘结连通(B > 0 共面连接)划分连通分量。"""
    cs = set(cells)
    seen, groups = set(), []
    for c in sorted(cs):
        if c in seen:
            continue
        comp, q = {c}, deque([c])
        seen.add(c)
        while q:
            a = q.popleft()
            for dx, dy, dz in DIRS6:
                n = (a[0] + dx, a[1] + dy, a[2] + dz)
                if n in cs and n not in seen and Beval.bond(a, n) > 0:
                    seen.add(n)
                    comp.add(n)
                    q.append(n)
        groups.append(sorted(comp))
    return groups


def cut_segments(cells, Beval, S):
    """REQ-010 §5.2:先粘结连通分组,再用世界对齐 S 格立方网格切,块内再按粘结连通。"""
    segs = []
    for grp in bond_groups(cells, Beval):
        by_chunk = {}
        for c in grp:
            by_chunk.setdefault((c[0] // S, c[1] // S, c[2] // S), []).append(c)
        for key in sorted(by_chunk):
            segs.extend(bond_groups(by_chunk[key], Beval))
    return segs


def origin_and_rho(cells):
    """§5.1/§5.3:原点 = 包围盒中心取整到最近格中心(同距取小);ρ = max|格心−原点| + 0.45√3。"""
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    zs = [c[2] for c in cells]

    def nearest(p):
        d = p - 0.5
        f = math.floor(d)
        return f if (d - f) <= 0.5 else f + 1

    oc = (nearest((min(xs) + max(xs) + 1) / 2.0),
          nearest((min(ys) + max(ys) + 1) / 2.0),
          nearest((min(zs) + max(zs) + 1) / 2.0))
    rmax = max(math.sqrt((c[0] - oc[0]) ** 2 + (c[1] - oc[1]) ** 2 + (c[2] - oc[2]) ** 2)
               for c in cells)
    return oc, rmax + CELL_HALF * math.sqrt(3.0)


# ---------------------------------------------------------------- rapier:地形

def exposed_faces(vox):
    """[(face_key,(ix,iy,iz)), ...];边界外按实心处理(不生成世界边界面)。"""
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


def slab_terrain(world, w, friction=0.8):
    """生产口径地形:暴露面薄片(半厚 0.075),全部挂在一个固定体上,
    每个 collider 是 primitive(**不是** Collider.compound,否则冲量读 0)。
    返回 (body_handle, solid_cell_set)。"""
    parts = []
    solid = set()
    for (axis, sign), (ix, iy, iz) in exposed_faces(w.vox):
        wx, wy, wz = ix + w.x0, iy + w.y0, iz + w.z0
        solid.add((wx, wy, wz))
        off = [0.5, 0.5, 0.5]
        half = [0.5, 0.5, 0.5]
        k = "xyz".index(axis)
        off[k] = 1.0 if sign > 0 else 0.0
        half[k] = HALF_SLAB
        parts.append(r.Collider.cuboid(half[0], half[1], half[2])
                     .translation((wx + off[0], wy + off[1], wz + off[2])))
    h = world.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED),
                       colliders=parts)
    return h, solid


def add_box(world, pos, half=(CELL_HALF,) * 3, fixed=False, density=1.0,
            friction=0.6, restitution=0.1, translation=None):
    """单个 cuboid 刚体(fixed=固定体,如垫块/墙)。translation 覆盖 pos。"""
    b = r.RigidBodyBuilder(r.RigidBodyType.FIXED if fixed else r.RigidBodyType.DYNAMIC)
    b = b.translation(tuple(translation if translation is not None else pos))
    return world.add_body(b, colliders=[r.Collider.cuboid(*half).density(density)
                                        .friction(friction).restitution(restitution)])


# ---------------------------------------------------------------- rapier:复合体

def compound_origin(cells):
    """cells: 世界整格列表 → (origin_cell, offsets)。原点规则同 origin_and_rho。"""
    oc, _ = origin_and_rho(cells)
    return oc, [(c[0] - oc[0], c[1] - oc[1], c[2] - oc[2]) for c in cells]


def merge_boxes(offsets):
    """把格偏移集合贪心合并成最少的长方体(半尺寸 = n·0.45 形式)。

    返回 [(center_offset, (hx,hy,hz)), ...](center/half 单位均为米,格心间距 1)。
    """
    rem = set(offsets)
    boxes = []
    while rem:
        c = min(rem)
        # 沿 x 延伸
        ex = 1
        while (c[0] + ex, c[1], c[2]) in rem:
            ex += 1
        # 沿 y 延伸(整行都在)
        ey = 1
        while all((c[0] + i, c[1] + ey, c[2]) in rem for i in range(ex)):
            ey += 1
        # 沿 z 延伸(整面都在)
        ez = 1
        while all((c[0] + i, c[1] + j, c[2] + ez) in rem
                  for i in range(ex) for j in range(ey)):
            ez += 1
        for i in range(ex):
            for j in range(ey):
                for k in range(ez):
                    rem.remove((c[0] + i, c[1] + j, c[2] + k))
        boxes.append(((c[0] + (ex - 1) / 2.0, c[1] + (ey - 1) / 2.0, c[2] + (ez - 1) / 2.0),
                      (ex * CELL_HALF, ey * CELL_HALF, ez * CELL_HALF)))
    return boxes


def add_compound(world, cells, origin_pos, rotation=None, merged=False,
                 density=1.0, friction=0.6, restitution=0.1):
    """一个动态刚体 + 每格一个 primitive cuboid(merged=True 时合并长方体)。

    cells: 世界整格列表(只用来算 offsets,不建体素);
    origin_pos: 原点(格中心)的世界位置;
    rotation: 缩放轴角向量(如 (0,0,0.349) 绕 z 转 20°)或 None。
    返回 (handle, origin_cell, offsets, cell_of_collider(dict handle->格序号))。
    """
    origin_cell, offsets = compound_origin(cells)
    if merged:
        # 合并长方体体积恰为格数 × 0.729(无缝拼合),密度 1 → 质量 = 格数 × 0.729
        parts = [r.Collider.cuboid(hx, hy, hz).translation(off)
                 .density(density).friction(friction).restitution(restitution)
                 for off, (hx, hy, hz) in merge_boxes(offsets)]
    else:
        parts = [r.Collider.cuboid(CELL_HALF, CELL_HALF, CELL_HALF).translation(off)
                 .density(density).friction(friction).restitution(restitution)
                 for off in offsets]
    b = r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC).translation(origin_pos)
    if rotation is not None:
        b = b.rotation(rotation)
    handle = world.add_body(b, colliders=parts)
    # collider 句柄 → 格序号:按世界位置匹配(生成时刻位姿已知)
    cell_of = {}
    body = world.rigid_bodies[handle]
    pos = np.array(tuple(body.translation), dtype=float)
    R = r_rotation(rotation) if rotation else np.eye(3)
    by_handle = {}
    for h in world.colliders.handles():
        c = world.colliders.get(h)
        if c.parent == handle:
            by_handle[h] = np.array(tuple(c.translation), dtype=float)
    for i, off in enumerate(offsets):
        expected = pos + R @ np.array(off, dtype=float)
        hbest = min(by_handle, key=lambda hh: float(np.linalg.norm(by_handle[hh] - expected)))
        cell_of[hbest] = i
    return handle, origin_cell, offsets, cell_of


def cell_world_pos(body, offsets):
    """各格中心的世界坐标 = body 位姿变换 offset。"""
    b = body
    t, rot = b.translation, b.rotation
    return [tuple(np.array(tuple(t), dtype=float)
                  + np.array(tuple(rot.transform_vector(off)), dtype=float))
            for off in offsets]


# ---------------------------------------------------------------- snap(§8.1)

def rotations24():
    """24 个轴对齐旋转(3×3 整数矩阵,列 = 带符号单位轴,det=+1),行主序展平。"""
    rots = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = [[0] * 3 for _ in range(3)]
            for i, p in enumerate(perm):
                m[i][p] = signs[i]
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                   - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            if det == 1:
                rots.append(m)
    return rots


ORIENT24 = rotations24()


def snap_pose(pos, rot):
    """吸附:朝向取 24 个轴对齐中最近(角度误差 = acos((tr(R·Rkᵀ)−1)/2)),
    原点取整到最近格中心(同距取小)。返回 (Rk, origin_cell, angle_err_rad)。"""
    M = np.array(rot.to_matrix()).reshape(3, 3)
    best, best_tr = None, -3.0
    for Rk in ORIENT24:
        tr = float(np.trace(M @ np.array(Rk).T))
        if tr > best_tr:
            best_tr, best = tr, Rk
    ang = math.acos(max(-1.0, min(1.0, (best_tr - 1.0) / 2.0)))
    oc = []
    for p in pos:
        d = p - 0.5
        f = math.floor(d)
        oc.append(int(f) if (d - f) <= 0.5 else int(f) + 1)
    return best, tuple(oc), ang


def snap_error_max(world, handle, origin_cell, offsets, Rk):
    b = world.rigid_bodies[handle]
    t = np.array(tuple(b.translation), dtype=float)
    rot = b.rotation
    Rk = np.array(Rk, dtype=float)
    errs = []
    for off in offsets:
        actual = t + np.array(tuple(rot.transform_vector(off)), dtype=float)
        target = np.array(origin_cell) + 0.5 + Rk @ np.array(off, dtype=float)
        errs.append(float(np.linalg.norm(actual - target)))
    return max(errs)


def r_rotation(rot_vec):
    """缩放轴角向量 → 3×3 旋转矩阵(local→world,与 rapier builder.rotation 一致)。"""
    axis = np.array(rot_vec, float)
    ang = float(np.linalg.norm(axis))
    if ang < 1e-12:
        return np.eye(3)
    k = axis / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


class SettleTracker:
    """凝固判据:线速度 < 0.1 且角速度 < 0.05 连续 20 拍,且最近 20 拍位移 ≤ 3 mm。"""

    def __init__(self, world, handles, lin=0.1, ang=0.05, need=20, max_disp=0.003):
        self.world = world
        self.handles = list(handles)
        self.lin, self.ang, self.need, self.max_disp = lin, ang, need, max_disp
        self.runs = {h: 0 for h in self.handles}
        self.prev = {h: None for h in self.handles}
        self.done = {}

    def step(self, tick):
        for h in self.handles:
            b = self.world.rigid_bodies[h]
            v, w, p = b.linvel, b.angvel, b.translation
            below = b.is_sleeping or (
                math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) < self.lin
                and math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2) < self.ang)
            if below and self.prev[h] is not None:
                d = math.dist(p, self.prev[h])
                if d > self.max_disp:
                    below = False
            self.prev[h] = tuple(p)
            self.runs[h] = self.runs[h] + 1 if below else 0
            if self.runs[h] >= self.need and h not in self.done:
                self.done[h] = tick
        return len(self.done)


def free_fall_pos(y0, t, g=9.81):
    return y0 - 0.5 * g * t * t
