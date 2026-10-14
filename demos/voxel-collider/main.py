"""Demo B:体素世界 → 物理 collider 的映射方案对比(架构验证,对应 REQ-001 激活层)。

既定共识:体素数组是唯一真相,collider 是派生缓存;暴露面(6 邻居判据)生成静态
cuboid;禁止用 Voxels 形状。本实验对比三种 collider 组织策略的成本。

体素坐标约定:格 (x,y,z) 占据 [x,x+1)^3,中心 (x+0.5,y+0.5,z+0.5);格宽 1。
暴露面薄片:厚度 0.15(半厚 0.075),跨在暴露面上(一半嵌入固体格)。

chunk 划分:x/z 方向 16×16 列、全高,一个 chunk 一个静态刚体(世界锚定坐标)。
运行:.venv/Scripts/python.exe demos/voxel-collider/main.py
"""
import time

import numpy as np
import rapier3d as r

SX, SY, SZ = 48, 32, 48
CHUNK = 16
GRAVITY = (0.0, -9.81, 0.0)
HALF_SLAB = 0.075
N_BALLS = 100
SETTLE = 100
BENCH = 300

# (法向轴, 符号) → 面中心偏移与半尺寸
FACE_GEO = {
    ("x", +1): ((1.0, 0.5, 0.5), (HALF_SLAB, 0.5, 0.5)),
    ("x", -1): ((0.0, 0.5, 0.5), (HALF_SLAB, 0.5, 0.5)),
    ("y", +1): ((0.5, 1.0, 0.5), (0.5, HALF_SLAB, 0.5)),
    ("y", -1): ((0.5, 0.0, 0.5), (0.5, HALF_SLAB, 0.5)),
    ("z", +1): ((0.5, 0.5, 1.0), (0.5, 0.5, HALF_SLAB)),
    ("z", -1): ((0.5, 0.5, 0.0), (0.5, 0.5, HALF_SLAB)),
}


def build_voxels():
    """确定性场景:起伏地形 + 一栋有门有内部的空心房(限制在 x>=28,不与落球区重叠)。"""
    x = np.arange(SX)
    z = np.arange(SZ)
    Xg, Zg = np.meshgrid(x, z, indexing="ij")
    h = (10 + 3 * np.sin(Xg * 0.3) * np.cos(Zg * 0.25) + 1.5 * np.sin(0.7 * Xg + 1.3)).astype(int)
    vox = np.zeros((SX, SY, SZ), dtype=bool)
    for y in range(SY):
        vox[:, y, :] = y < h
    # 空心房:外壳实心,掏空内部,留 1 格厚墙,顶层当屋顶
    hx0, hx1, hz0, hz1 = 28, 37, 28, 37
    base = int(h[hx0:hx1 + 1, hz0:hz1 + 1].max())
    vox[hx0:hx1 + 1, base:base + 5, hz0:hz1 + 1] = True
    vox[hx0 + 1:hx1, base + 1:base + 4, hz0 + 1:hz1] = False
    vox[hx0 + 2:hx0 + 4, base:base + 2, hz0] = False  # 门洞
    return vox, h


def exposed_faces(vox):
    """返回 [(face_key, cell(x,y,z)), ...],face_key=(axis, sign)。"""
    P = np.pad(vox, 1)  # 外圈空气
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
        for x, y, z in zip(*np.nonzero(m)):
            faces.append((key, (int(x), int(y), int(z))))
    return faces


def face_collider(face_key, cell):
    """单个暴露面 → (世界中心, 半尺寸)。"""
    (axis, sign), (x, y, z) = face_key, cell
    off, half = FACE_GEO[(axis, sign)]
    center = (x + off[0], y + off[1], z + off[2])
    return center, half


def merge_rects(mask):
    """2D 贪心矩形合并:行内扩 run,再向下扩等宽 run。返回 (r1,c1,r2,c2) 含端点。"""
    used = np.zeros_like(mask)
    rects = []
    h, w = mask.shape
    for i in range(h):
        j = 0
        while j < w:
            if mask[i, j] and not used[i, j]:
                j2 = j
                while j2 + 1 < w and mask[i, j2 + 1] and not used[i, j2 + 1]:
                    j2 += 1
                i2 = i
                while i2 + 1 < h and mask[i2 + 1, j:j2 + 1].all() and not used[i2 + 1, j:j2 + 1].any():
                    i2 += 1
                used[i:i2 + 1, j:j2 + 1] = True
                rects.append((i, j, i2, j2))
                j = j2 + 1
            else:
                j += 1
    return rects


def merged_face_colliders(faces):
    """暴露面按(法向,平面)分组后做 2D 矩形合并,返回 [(center, half)]。"""
    groups = {}
    for (axis, sign), (x, y, z) in faces:
        plane = {"x": (x + 1) if sign > 0 else x,
                 "y": (y + 1) if sign > 0 else y,
                 "z": (z + 1) if sign > 0 else z}[axis]
        groups.setdefault((axis, sign, plane), set()).add((x, y, z))
    out = []
    for (axis, sign, plane), cells in groups.items():
        # 切向两轴;法向轴的坐标全部相同
        tan = [a for a in ("x", "y", "z") if a != axis]
        idx = {a: i for i, a in enumerate("xyz")}
        a1, a2 = tan
        d1 = {"x": SX, "y": SY, "z": SZ}[a1]
        d2 = {"x": SX, "y": SY, "z": SZ}[a2]
        m = np.zeros((d1, d2), dtype=bool)
        for (x, y, z) in cells:
            m[(x, y, z)[idx[a1]], (x, y, z)[idx[a2]]] = True
        off, half = FACE_GEO[(axis, sign)]
        for (r1, c1, r2, c2) in merge_rects(m):
            center = [0.0, 0.0, 0.0]
            halfv = [0.0, 0.0, 0.0]
            center[idx[axis]] = plane + (0.5 if sign > 0 else 0.0)
            halfv[idx[axis]] = HALF_SLAB
            center[idx[a1]] = r1 + (r2 - r1 + 1) / 2
            halfv[idx[a1]] = (r2 - r1 + 1) / 2
            center[idx[a2]] = c1 + (c2 - c1 + 1) / 2
            halfv[idx[a2]] = (c2 - c1 + 1) / 2
            out.append((tuple(center), tuple(halfv)))
    return out


def chunk_of(cell):
    return (cell[0] // CHUNK, cell[2] // CHUNK)


def build_strategy(faces, mode):
    """mode: per-face / merged / compound。返回 {chunk_key: add_body 所需 collider 列表}。"""
    chunks = {}
    if mode == "merged":
        # 先按 chunk 分组、组内合并:保证每个 cuboid 不跨 chunk,换装边界干净
        by_chunk = {}
        for (key, cell) in faces:
            by_chunk.setdefault(chunk_of(cell), []).append((key, cell))
        for k, chunk_faces in by_chunk.items():
            chunks[k] = merged_face_colliders(chunk_faces)
        return {k: [r.Collider.cuboid(*h).translation(c).friction(0.8).restitution(0.0)
                    for (c, h) in v] for k, v in chunks.items()}
    # per-face 与 compound 都逐面处理
    per_chunk = {}
    for (key, cell) in faces:
        center, half = face_collider(key, cell)
        per_chunk.setdefault(chunk_of(cell), []).append((center, half))
    if mode == "per-face":
        return {k: [r.Collider.cuboid(*h).translation(c).friction(0.8).restitution(0.0)
                    for (c, h) in v] for k, v in per_chunk.items()}
    if mode == "compound":
        out = {}
        for k, faces_list in per_chunk.items():
            parts = [(r.Isometry3.from_translation(*c), r.SharedShape.cuboid(*h))
                     for (c, h) in faces_list]
            out[k] = [r.Collider.compound(parts).friction(0.8).restitution(0.0)]
        return out
    raise ValueError(mode)


def run_strategy(mode, vox, h):
    t0 = time.perf_counter()
    faces = exposed_faces(vox)
    t_extract = time.perf_counter() - t0

    t0 = time.perf_counter()
    chunk_colliders = build_strategy(faces, mode)
    t_build = time.perf_counter() - t0

    world = r.PhysicsWorld(GRAVITY)
    t0 = time.perf_counter()
    bodies = {}
    for key, cols in sorted(chunk_colliders.items()):
        bodies[key] = world.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, 0.0, 0.0)),
            colliders=cols,
        )
    t_insert = time.perf_counter() - t0

    rng = np.random.default_rng(42)
    handles = []
    for _ in range(N_BALLS):
        bx = float(rng.uniform(2, 26))
        bz = float(rng.uniform(2, 26))
        handles.append(world.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
            .translation((bx, 40.0 + float(rng.uniform(0, 5)), bz))
            .linvel((float(rng.uniform(-1, 1)), 0.0, float(rng.uniform(-1, 1)))),
            colliders=[r.Collider.ball(0.4).friction(0.5).restitution(0.2).density(1.0)],
        ))

    for _ in range(SETTLE):
        world.step()
    t0 = time.perf_counter()
    for _ in range(BENCH):
        world.step()
    step_ms = (time.perf_counter() - t0) * 1000 / BENCH

    # 穿深检查:抽样 5 球,应停在 terrai 表面附近(顶部平面 y=h)
    worst_pen = 0.0
    for hd in handles[::20]:
        b = world.rigid_bodies[hd]
        bx, by, bz = (float(v) for v in b.translation)
        ground_h = h[int(np.clip(bx, 0, SX - 1)), int(np.clip(bz, 0, SZ - 1))]
        worst_pen = max(worst_pen, (ground_h + 0.4) - by)  # 球心应 >= 地面+半径

    n_colliders = sum(len(v) for v in chunk_colliders.values())
    return {
        "mode": mode,
        "faces": len(faces),
        "colliders": n_colliders,
        "extract_ms": t_extract * 1000,
        "build_ms": t_build * 1000,
        "insert_ms": t_insert * 1000,
        "step_ms": step_ms,
        "worst_pen": worst_pen,
        "_bodies": bodies,
        "_world": world,
        "_chunk_colliders": chunk_colliders,
    }


def swap_test(res):
    """换装测试:卸载 5 个 chunk 再装载(用缓存的 collider 列表)。"""
    world, bodies, cols = res["_world"], res["_bodies"], res["_chunk_colliders"]
    keys = sorted(bodies)[:5]
    t0 = time.perf_counter()
    for k in keys:
        world.remove_body(bodies[k])
    t_unload = (time.perf_counter() - t0) * 1000
    t0 = time.perf_counter()
    for k in keys:
        bodies[k] = world.add_body(
            r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, 0.0, 0.0)),
            colliders=cols[k],
        )
    t_load = (time.perf_counter() - t0) * 1000
    return t_unload, t_load


if __name__ == "__main__":
    vox, h = build_voxels()
    solid_n = int(vox.sum())
    print(f"体素场 {SX}x{SY}x{SZ},solid={solid_n} 格({solid_n / (SX*SY*SZ):.1%})")
    faces = exposed_faces(vox)
    print(f"暴露面总数:{len(faces)}")
    merged = merged_face_colliders(faces)
    print(f"贪心合并后 cuboid 数:{len(merged)}(压缩比 {len(faces) / max(len(merged),1):.2f}x)")
    del faces, merged

    results = {}
    for mode in ("per-face", "merged", "compound"):
        res = run_strategy(mode, vox, h)
        results[mode] = res
        print(f"\n===== 策略:{mode} =====")
        print(f"collider/形状数:{res['colliders']}")
        print(f"面提取 {res['extract_ms']:.0f}ms | collider 构建(Python){res['build_ms']:.0f}ms | "
              f"插入物理世界 {res['insert_ms']:.0f}ms")
        print(f"稳定后 step 均耗时(100 球 + 全部静态 collider):{res['step_ms']:.3f} ms")
        print(f"抽样球最大穿深:{res['worst_pen']:.4f} m(负值=悬空,应接近 0)")

    u, l = swap_test(results["per-face"])
    print(f"\n===== 换装测试(per-face,5 个 chunk) =====")
    print(f"卸载 {u:.1f}ms | 装载 {l:.1f}ms(含 Python 侧 add_body)")
