"""demos/terrain-destruction 共用工具(REQ-009 任务 00 demo)。

坐标约定:格 (x,y,z) 占据 [x,x+1)^3,中心 (x+0.5,y+0.5,z+0.5);地表 y=0。
体素场 vox 为 np 数组,索引 vox[ix,iy,iz];ix=0 对应世界 x = x0(x0 可为负)。
地形碰撞体按生产口径:暴露面薄片(半厚 0.075,跨在面上),按 16×16 chunk 分组成
固定复合体(见 demos/voxel-collider/main.py)。
"""
import math

import numpy as np
import rapier3d as r

AIR, STONE, DIRT, BEDROCK, SAND = 0, 1, 2, 3, 4
BLOCK_CHAR = {AIR: " ", STONE: "#", DIRT: "=", BEDROCK: "B", SAND: "."}
INF_R = 1e12          # bedrock 的数值化 ∞(任何 power 都打不过)
HALF_SLAB = 0.075     # 暴露面薄片半厚(生产口径)
GRAVITY = (0.0, -9.81, 0.0)
DT = 1.0 / 60.0
CHUNK = 16

FACE_GEO = {
    ("x", +1): ((1.0, 0.5, 0.5), (HALF_SLAB, 0.5, 0.5)),
    ("x", -1): ((0.0, 0.5, 0.5), (HALF_SLAB, 0.5, 0.5)),
    ("y", +1): ((0.5, 1.0, 0.5), (0.5, HALF_SLAB, 0.5)),
    ("y", -1): ((0.5, 0.0, 0.5), (0.5, HALF_SLAB, 0.5)),
    ("z", +1): ((0.5, 0.5, 1.0), (0.5, 0.5, HALF_SLAB)),
    ("z", -1): ((0.5, 0.5, 0.0), (0.5, 0.5, HALF_SLAB)),
}


def segment_cells(p0, p1):
    """Amanatides–Woo 体素遍历:p0→p1 线段依次穿过的格,含两端所在格,按穿过顺序。

    p0/p1 为 float 三元组;返回 [(x,y,z), ...]。
    """
    cx, cy, cz = int(math.floor(p0[0])), int(math.floor(p0[1])), int(math.floor(p0[2]))
    ex, ey, ez = int(math.floor(p1[0])), int(math.floor(p1[1])), int(math.floor(p1[2]))
    dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    cells = [(cx, cy, cz)]
    guard = abs(ex - cx) + abs(ey - cy) + abs(ez - cz) + 3
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    sz = 1 if dz > 0 else -1

    def tmax(p, c, d, s):
        if d == 0.0:
            return math.inf
        boundary = float(c + 1) if s > 0 else float(c)
        return (boundary - p) / d

    tmx = tmax(p0[0], cx, dx, sx)
    tmy = tmax(p0[1], cy, dy, sy)
    tmz = tmax(p0[2], cz, dz, sz)
    tdx = abs(1.0 / dx) if dx != 0.0 else math.inf
    tdy = abs(1.0 / dy) if dy != 0.0 else math.inf
    tdz = abs(1.0 / dz) if dz != 0.0 else math.inf
    for _ in range(guard):
        if (cx, cy, cz) == (ex, ey, ez):
            return cells
        if tmx <= tmy and tmx <= tmz:
            cx += sx
            tmx += tdx
        elif tmy <= tmz:
            cy += sy
            tmy += tdy
        else:
            cz += sz
            tmz += tdz
        cells.append((cx, cy, cz))
    raise RuntimeError("segment_cells: guard 触发(遍历未收敛)")


class BlastField:
    """给定体素场与爆心,预计算 rmax 内全部实心目标格的射线,供参数扫描反复求值。

    射线 = 爆心→格中心 的 segment_cells 序列(末元素即目标格自身,求衰减时排除)。
    """

    def __init__(self, vox, x0, y0, z0, blast, rmax=12.0, surface_y=-1):
        self.vox = vox
        self.x0, self.y0, self.z0 = x0, y0, z0
        self.blast = blast
        self.surface_y = surface_y
        solid = np.argwhere(vox != AIR)
        centers = solid + 0.5 + np.array([x0, y0, z0])
        d = np.linalg.norm(centers - np.array(blast), axis=1)
        keep = d < rmax
        solid = solid[keep]
        d = d[keep]
        order = np.lexsort((solid[:, 2], solid[:, 1], solid[:, 0]))  # BlockPos 序,确定性
        solid = solid[order]
        d = d[order]
        paths = [segment_cells(blast, tuple(c + 0.5 + np.array([x0, y0, z0]))) for c in solid]
        lmax = max(len(p) for p in paths)
        n = len(paths)
        ids = np.zeros((n, lmax), dtype=np.int64)
        lens = np.zeros(n, dtype=np.int64)
        for i, p in enumerate(paths):
            lens[i] = len(p)
            for j, (x, y, z) in enumerate(p):
                ids[i, j] = (x - x0) * vox.shape[1] * vox.shape[2] + (y - y0) * vox.shape[2] + (z - z0)
        self.ids, self.lens, self.dists = ids, lens, d
        self.coords = solid  # 场内索引坐标
        # 各扫描半径下,爆心球内与体素场相交的全部格数(求值“访问格数”口径)
        gx = np.arange(x0, x0 + vox.shape[0]) + 0.5
        gy = np.arange(y0, y0 + vox.shape[1]) + 0.5
        gz = np.arange(z0, z0 + vox.shape[2]) + 0.5
        dd = np.sqrt((gx[:, None, None] - blast[0]) ** 2
                     + (gy[None, :, None] - blast[1]) ** 2
                     + (gz[None, None, :] - blast[2]) ** 2)
        self.visited_by_r = {rr: int((dd < rr).sum()) for rr in (4, 8, 12)}

    def evaluate(self, res_of_block, power, radius, k, att, model="ray"):
        """对一组爆炸参数求值。res_of_block: dict block_id -> R。

        返回 dict(destroyed/detached/pulverized/depth/diameter/...);
        cells: 场内索引 (ix,iy,iz) -> 1=脱落 / 2=粉碎(仅被破坏的格)。
        """
        vox = self.vox
        resmap = np.array([res_of_block.get(b, INF_R) for b in range(5)])
        cellres = resmap[vox.ravel()]
        sel = self.dists < radius
        ids, lens, dist = self.ids[sel], self.lens[sel], self.dists[sel]
        coords = self.coords[sel]
        p0 = power * (1.0 - dist / radius)
        if model == "ray":
            path = cellres[ids]
            last = lens - 1
            tres = path[np.arange(len(ids)), last]
            mask = np.arange(ids.shape[1])[None, :] < last[:, None]
            loss = (path * mask).sum(axis=1)
            P = p0 - att * loss
        else:
            tres = cellres[ids[np.arange(len(ids)), lens - 1]]
            P = p0
        hit = P > tres
        pulv = hit & (P > k * tres)
        n_hit = int(hit.sum())
        n_pulv = int(pulv.sum())
        cells = {}
        wx = coords[:, 0] + self.x0
        wy = coords[:, 1] + self.y0
        wz = coords[:, 2] + self.z0
        for i in np.nonzero(hit)[0]:
            cells[tuple(coords[i])] = 2 if pulv[i] else 1
        hit_idx = np.nonzero(hit)[0]
        depth = int((-wy[hit_idx]).max()) if n_hit else 0
        # 直径:地表层被破坏格在 x、z 两个方向跨度较大者
        diam = 0
        if n_hit:
            surf = hit_idx[wy[hit_idx] == self.surface_y]
            if len(surf):
                diam = int(max(wx[surf].max() - wx[surf].min() + 1,
                               wz[surf].max() - wz[surf].min() + 1))
        return {
            "destroyed": n_hit,
            "detached": n_hit - n_pulv,
            "pulverized": n_pulv,
            "depth": depth,
            "diameter": diam,
            "cells": cells,
            "targets": int(sel.sum()),
            "visited": self.visited_by_r.get(int(radius), 0),
            "ray_steps": int((lens - 1).sum()),
        }


def render_slice(vox, x0, y0, z0, cells, z_world, top, bottom):
    """过 z_world 的 x–y 剖面 ASCII 图。cells: (ix,iy,iz)->1/2(脱落/粉碎)。"""
    iz = z_world - z0
    rows = []
    for y in range(top, bottom - 1, -1):
        iy = y - y0
        row = []
        for ix in range(vox.shape[0]):
            st = cells.get((ix, iy, iz))
            if st == 1:
                row.append("o")
            elif st == 2:
                row.append("x")
            else:
                row.append(BLOCK_CHAR[int(vox[ix, iy, iz])])
        rows.append(f"{y:>4} " + "".join(row))
    rows.append("     " + "".join(str(abs(x0 + ix) % 10) for ix in range(vox.shape[0])))
    return "\n".join(rows)


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


def terrain_bodies(world, vox, x0, y0, z0):
    """生产口径地形碰撞体:暴露面薄片,按 16×16(xz) chunk 组成固定复合体。

    返回 {chunk_key: handle}。
    """
    bodies = {}
    by_chunk = {}
    for key, (ix, iy, iz) in exposed_faces(vox):
        wx, wy, wz = ix + x0, iy + y0, iz + z0
        by_chunk.setdefault((wx // CHUNK, wz // CHUNK), []).append((key, (wx, wy, wz)))
    for ck in sorted(by_chunk):
        parts = []
        for (axis, sign), (wx, wy, wz) in by_chunk[ck]:
            (ox, oy, oz), (hx, hy, hz) = FACE_GEO[(axis, sign)]
            parts.append((r.Isometry3.from_translation(wx + ox, wy + oy, wz + oz),
                          r.SharedShape.cuboid(hx, hy, hz)))
        bodies[ck] = world.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.FIXED),
            colliders=[r.Collider.compound(parts).friction(0.8).restitution(0.0)],
        )
    return bodies


def flat_ground(nx, nz, dirt_layers=3, stone_layers=0):
    """超平坦:最底层 bedrock,其上 dirt_layers 层 dirt(stone_layers>0 时改 stone)。

    地表 y=0;返回 (vox, x0, y0, z0)。
    """
    ny = dirt_layers + 2
    vox = np.zeros((nx, ny, nz), dtype=np.int8)
    vox[:, -1, :] = BEDROCK
    vox[:, :-1, :] = DIRT if not stone_layers else STONE
    return vox, 0, -dirt_layers - 1, 0


class RestTracker:
    """凝固静止判据:线速度 < 0.1 m/s 且角速度 < 0.05 rad/s,连续 need 拍。

    睡眠中的体视为满足。记录每个体首次“连续满足满 need 拍”的拍号与全体就绪拍。
    """

    def __init__(self, world, handles, lin=0.1, ang=0.05, need=20):
        self.world = world
        self.handles = list(handles)
        self.lin, self.ang, self.need = lin, ang, need
        self.runs = {h: 0 for h in self.handles}
        self.done = {}

    def step(self, tick):
        for h in self.handles:
            b = self.world.rigid_bodies[h]
            v = b.linvel
            w = b.angvel
            below = b.is_sleeping or (
                math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) < self.lin
                and math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2) < self.ang)
            self.runs[h] = self.runs[h] + 1 if below else 0
            if self.runs[h] >= self.need and h not in self.done:
                self.done[h] = tick
        return len(self.done)

    @property
    def settled_tick(self):
        return max(self.done.values()) if len(self.done) == len(self.handles) else None
