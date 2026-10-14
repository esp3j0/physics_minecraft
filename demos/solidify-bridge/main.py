# -*- coding: utf-8 -*-
"""
凝固桥(solidify bridge)可行性验证:体素层 <-> 实体层
======================================================

要验证的架构共识:
    动态体睡觉 -> snap 去旋转 -> 写体素数组 -> 换静态 collider -> 删刚体。
本脚本在 rapier3d 0.36.0 上实测该流程的可行性、误差与耗时,并对比三种触发策略:

    P1 等睡眠 :body.is_sleeping == True 后触发(等完全睡眠);
    P2 标准   :线速度 < 0.1 且角速度 < 0.05 连续 20 tick 即触发(不等完全睡眠);
    P2 宽松   :线速度 < 0.3 且角速度 < 0.5 连续 10 tick(对照实验,用于拉出
               "更早触发 vs 更大 snap 误差"的权衡曲线;相对任务书的改动,见 README)。

场景与坐标约定
--------------
* 地面:FIXED cuboid(50, 0.5, 50) @ (0, -0.5, 0),顶面 y = 0。
* 立方体:1x1x1 动态(半边长 0.5),从 (0, 10, 0) 落下,
  初线速度 (0.5, 0, 0)、初角速度 (2.0, 1.0, 0.5),落地后翻滚直至静止。
* 体素约定:**格点在整数坐标**,格 (i,j,k) 覆盖 [i,i+1) x [j,j+1) x [k,k+1),
  体素中心 = (i+0.5, j+0.5, k+0.5);地面顶面 y=0,故第一层体素中心 y=0.5。
  - snap 位置:质心每个分量取 cell = floor(c),吸附到 cell + 0.5(最近体素中心);
  - snap 旋转:在 24 个轴对齐朝向(行列均为 ±单位向量的符号置换矩阵,det=+1)
    中取与当前姿态转角最小者(numpy 构造 + arccos((trace-1)/2))。
* 桥操作(触发后一次完成,单独计时):
    读 translation/rotation -> snap 去旋转 -> snap 位置 ->
    remove 动态体 -> 同位置 add FIXED 体 + Collider.cuboid(0.5,0.5,0.5) ->
    把格号写入 set(体素数组的占位表示)。

触发后的验证
------------
从块顶上方 5 格(球底距块顶 5 格)掉一个 r=0.5 的球,带 0.05 m/s 微水平速度、
角阻尼 2.0(本绑定未暴露滚动摩擦,用角阻尼近似"滚一段后停下"),跑 600 tick,检查:
    1) 球稳定停在块上(末 100 tick 平均速率 < 0.05,且水平仍在块的正上方);
    2) 无穿模:末 100 tick 平均球底 >= 块顶 - 0.01 m;
    3) 位置有限:全程无 NaN、无爆炸(|p| 不发散)。

运行:C:\\Users\\esp3j\\rep\\physics_minecraft\\.venv\\Scripts\\python.exe main.py
输出:纯文本/Markdown 报告(不画图)。可重复运行,物理确定、结果稳定。
"""

import itertools
import math
import sys
import time

import numpy as np
import rapier3d as r

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ---------------- 常量 ----------------
GRAVITY = (0.0, -9.81, 0.0)
CUBE_HALF = 0.5
GROUND_TOP = 0.0                 # 地面顶面 y
FRICTION = 0.7

# P2 速度阈值策略参数:标准组用任务书建议值;另加一组"宽松"对照
# (P2-loose:角阈值取引擎睡眠阈值 0.5、只等 10 tick),用来拉出"更早触发 vs 更大误差"的权衡曲线
STRATEGIES = [
    # (标签, 是否等睡眠, 线速度阈值, 角速度阈值, 连续 tick 数)
    ("P1 等睡眠", True, None, None, None),
    ("P2 标准", False, 0.10, 0.05, 20),
    ("P2 宽松", False, 0.30, 0.50, 10),
]

MAX_WAIT_TICKS = 3000            # 触发等待上限(超时强制触发并标记)

BALL_GAP = 5.0                   # 球底距块顶 5 格
VERIFY_TICKS = 600
STABLE_SPEED = 0.05              # 末 100 tick 平均速率阈值(m/s)
PEN_TOL = 0.01                   # 穿深容差(m)
ON_BLOCK_R = 0.45                # 球心仍在块心水平 0.45 内算"停在块上"


def as_arr(v):
    """Vec3 -> numpy(兼容迭代或 x/y/z 属性两种实现)。"""
    try:
        return np.asarray(v, dtype=float)
    except Exception:
        return np.array([v.x, v.y, v.z], dtype=float)


def norm3(v):
    a = as_arr(v)
    return float(np.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]))


# ---------------- 24 个轴对齐朝向 ----------------
def build_24_rotations():
    """符号置换矩阵:每行/列恰一个 ±1,det=+1 的恰有 24 个 = 立方体的旋转群。"""
    rots = []
    for perm in itertools.permutations(range(3)):
        for sx in (1.0, -1.0):
            for sy in (1.0, -1.0):
                for sz in (1.0, -1.0):
                    R = np.zeros((3, 3))
                    for row, s in enumerate((sx, sy, sz)):
                        R[row, perm[row]] = s
                    if round(float(np.linalg.det(R))) == 1:
                        rots.append(R)
    assert len(rots) == 24
    return rots


ROTS24 = build_24_rotations()


def rotation_matrix(body):
    """读 body.rotation(Rotation3) -> 3x3 旋转矩阵(to_matrix 是行主序 9 元展平)。"""
    return np.array(body.rotation.to_matrix(), dtype=float).reshape(3, 3)


def rel_angle_deg(Ra, Rb):
    """两个旋转之间的转角(度),arccos((trace(Ra^T Rb)-1)/2)。"""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def snap_rotation(R_pre):
    """在 24 个轴对齐朝向里取与 R_pre 夹角最小者,返回(矩阵, 角度误差度)。"""
    best, best_err = None, 1e18
    for C in ROTS24:
        err = rel_angle_deg(C, R_pre)
        if err < best_err:
            best, best_err = C, err
    return best, best_err


def mat_to_scaled_axis(R):
    """旋转矩阵 -> 缩放轴角向量(轴*弧度)。builder.rotation() 只吃这个格式(实测)。"""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if ang < 1e-9:
        return (0.0, 0.0, 0.0)
    if abs(math.pi - ang) < 1e-6:
        # 180 度时 sin=0,从对角元提取轴:R = 2*n*n^T - I
        i = int(np.argmax(np.diag(R)))
        n = np.zeros(3)
        n[i] = math.sqrt(max(0.0, (R[i, i] + 1.0) / 2.0))
        j, k = [t for t in range(3) if t != i]
        if abs(n[i]) > 1e-9:
            n[j] = R[j, i] / (2.0 * n[i])
            n[k] = R[k, i] / (2.0 * n[i])
        return tuple(n * ang)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis /= (2.0 * math.sin(ang))
    return tuple(axis * ang)


def snap_to_voxel_center(t):
    """位置吸附:每个分量 cell=floor(c),吸附到 cell+0.5(最近体素中心)。"""
    cells = [math.floor(c) for c in t]
    center = np.array([c + 0.5 for c in cells], dtype=float)
    return center, tuple(cells)


# ---------------- 场景 ----------------
def make_world():
    world = r.PhysicsWorld(GRAVITY)  # 重力必须在构造时显式给(默认 0)
    world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, -0.5, 0.0)),
        colliders=[r.Collider.cuboid(50.0, 0.5, 50.0).friction(FRICTION)],
    )
    return world


def spawn_cube(world):
    """1x1x1 动态立方体:空中落下 + 翻滚。"""
    return world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation((0.0, 10.0, 0.0))
        .linvel((0.5, 0.0, 0.0))
        .angvel((2.0, 1.0, 0.5))
        .can_sleep(True),
        colliders=[r.Collider.cuboid(CUBE_HALF, CUBE_HALF, CUBE_HALF)
                   .friction(FRICTION).restitution(0.0).density(1.0)],
    )


# ---------------- 触发等待 ----------------
def wait_until_trigger(world, handle, use_sleep, lin_t, ang_t, consec_need):
    """逐步模拟直到触发。返回(触发 tick, 首次进入该策略慢速区 tick, 触发时线/角速度,
    是否超时, 平均步耗时ms, 触发时是否已睡眠)。"""
    consec = 0
    first_slow = None
    step_ns = 0
    lin = ang = 0.0
    trigger_tick = MAX_WAIT_TICKS
    timeout = True
    for tick in range(1, MAX_WAIT_TICKS + 1):
        t0 = time.perf_counter_ns()
        world.step()
        step_ns += time.perf_counter_ns() - t0

        body = world.rigid_bodies[handle]
        lin, ang = norm3(body.linvel), norm3(body.angvel)
        slow = (lin < lin_t and ang < ang_t) if lin_t is not None else False
        if slow and first_slow is None:
            first_slow = tick

        if use_sleep:
            if body.is_sleeping:          # P1:完全睡眠才触发
                trigger_tick, timeout = tick, False
                break
        else:
            consec = consec + 1 if slow else 0
            if consec >= consec_need:     # P2:连续 N tick 低速即触发
                trigger_tick, timeout = tick, False
                break
    sleeping = bool(world.rigid_bodies[handle].is_sleeping)
    avg_step_ms = step_ns / max(1, trigger_tick) / 1e6
    return trigger_tick, first_slow, lin, ang, timeout, avg_step_ms, sleeping


# ---------------- 桥操作 ----------------
def solidify(world, handle, voxels):
    """凝固桥:读态 -> snap 去旋转 -> snap 位置 -> 换静态体 -> 写体素 set。"""
    body = world.rigid_bodies[handle]
    t_pre = as_arr(body.translation)
    R_pre = rotation_matrix(body)

    # 附加诊断:物理姿态(可能带倾斜)的最低角相对地面的陷入深度(负=陷入)
    corner_min_y = min(float((t_pre + R_pre @ np.array([dx, dy, dz]))[1])
                       for dx in (-CUBE_HALF, CUBE_HALF)
                       for dy in (-CUBE_HALF, CUBE_HALF)
                       for dz in (-CUBE_HALF, CUBE_HALF))
    pre_pen = GROUND_TOP - corner_min_y        # >0 表示物理体已有角陷入地面(snap 会一并修正)

    C, ang_err = snap_rotation(R_pre)          # 1) snap 旋转:最小转角对齐
    t_snap, cell = snap_to_voxel_center(t_pre)  # 2) snap 位置:最近体素中心
    disp = float(np.linalg.norm(t_snap - t_pre))

    t0 = time.perf_counter_ns()                 # ---- 桥耗时开始 ----
    world.remove_body(handle)                   # 3) 删动态刚体
    sa = mat_to_scaled_axis(C)                  #    snapped 朝向(轴对齐立方体等价于恒等,仍如实写回)
    builder = r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation(tuple(t_snap))
    if float(np.linalg.norm(sa)) > 1e-12:
        builder = builder.rotation(sa)
    world.add_body(
        builder,
        colliders=[r.Collider.cuboid(CUBE_HALF, CUBE_HALF, CUBE_HALF)
                   .friction(FRICTION).density(1.0)],
    )
    voxels.add(cell)                            # 4) 写体素数组(set 占位)
    bridge_ms = (time.perf_counter_ns() - t0) / 1e6  # ---- 桥耗时结束 ----

    return {
        "t_pre": t_pre, "R_pre": R_pre, "t_snap": t_snap, "cell": cell,
        "disp": disp, "disp_vec": t_snap - t_pre, "ang_err": ang_err,
        "pre_pen": pre_pen, "bridge_ms": bridge_ms,
    }


# ---------------- 后续验证 ----------------
def drop_ball_and_verify(world, block_center):
    """从块顶上方 5 格掉球,600 tick 后检查稳定/穿模/有限性。"""
    top = block_center[1] + CUBE_HALF
    bx, bz = block_center[0], block_center[2]
    handle = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation((bx, top + BALL_GAP + CUBE_HALF, bz))  # 球心:块顶 + 5格 + 半径
        .linvel((0.05, 0.0, 0.0))
        .angular_damping(2.0)                                # 无滚动摩擦,用角阻尼让球"滚一段后停"
        .can_sleep(True),
        colliders=[r.Collider.ball(CUBE_HALF).friction(FRICTION).restitution(0.05).density(1.0)],
    )

    ys, speeds = [], []
    finite = True
    peak_abs = 0.0
    for _ in range(VERIFY_TICKS):
        world.step()
        b = world.rigid_bodies[handle]
        p = as_arr(b.translation)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(as_arr(b.linvel)))):
            finite = False
            peak_abs = float("inf")
            break
        peak_abs = max(peak_abs, float(np.abs(p).max()))
        ys.append(float(p[1]))
        speeds.append(norm3(b.linvel))

    end = world.rigid_bodies[handle]
    p_end = as_arr(end.translation)
    n = min(100, len(ys))
    avg_y = sum(ys[-n:]) / n
    avg_speed = sum(speeds[-n:]) / n
    ball_bottom = avg_y - CUBE_HALF
    penetration = top - ball_bottom            # >0:球陷入块顶 penetration 米
    stable = avg_speed < STABLE_SPEED
    drift = math.hypot(float(p_end[0]) - bx, float(p_end[2]) - bz)
    on_block = drift <= ON_BLOCK_R
    return {
        "avg_speed100": avg_speed, "penetration": penetration, "finite": finite,
        "peak_abs": peak_abs, "stable": stable, "drift": drift, "on_block": on_block,
        "ball_end_y": float(p_end[1]), "block_top": top,
        "ok": stable and on_block and finite and penetration <= PEN_TOL,
    }


# ---------------- 单策略完整流程 ----------------
def run_strategy(label, use_sleep, lin_t, ang_t, consec_need):
    print("\n" + "=" * 64)
    print("策略 %s:%s" % (label, "等睡眠(is_sleeping)" if use_sleep
                          else "速度阈值(lin<%.2f 且 ang<%.2f 连续 %d tick)"
                               % (lin_t, ang_t, consec_need)))
    print("=" * 64)

    t_wall = time.perf_counter()
    world = make_world()
    voxels = set()
    handle = spawn_cube(world)

    tick, first_slow, lin, ang, timeout, step_ms, sleeping_at = wait_until_trigger(
        world, handle, use_sleep, lin_t, ang_t, consec_need)
    print("触发 @ tick %d(%.2f s)%s" % (tick, tick / 60.0, "  [超时!]" if timeout else ""))
    print("  触发时:is_sleeping=%s lin=%.4f m/s ang=%.4f rad/s" % (sleeping_at, lin, ang))
    if lin_t is not None:
        print("  首次进入本策略慢速区(lin<%.2f & ang<%.2f)@ tick %s" % (lin_t, ang_t, first_slow))

    rec = solidify(world, handle, voxels)
    print("  snap 前质心: (%.4f, %.4f, %.4f)" % tuple(rec["t_pre"]))
    print("  snap 后质心: (%.4f, %.4f, %.4f)  -> 体素格 %s" % (rec["t_snap"][0], rec["t_snap"][1], rec["t_snap"][2], rec["cell"]))
    print("  snap 位移: %.3f mm(竖直分量 %+.3f mm)| snap 角度误差: %.4f deg"
          % (rec["disp"] * 1000, rec["disp_vec"][1] * 1000, rec["ang_err"]))
    print("  snap 前物理姿态角陷入地面: %.3f mm(由 snap 一并修正)" % (rec["pre_pen"] * 1000))

    block_bottom = rec["t_snap"][1] - CUBE_HALF
    ground_pen = GROUND_TOP - block_bottom      # >0 表示块陷入地面
    print("  块底 y=%.4f vs 地面顶 y=%.1f -> 穿深 %.3f mm | 体素数组=%s" %
          (block_bottom, GROUND_TOP, ground_pen * 1000, sorted(voxels)))
    print("  桥耗时(remove+add+写体素): %.3f ms | 等待期平均 step: %.4f ms/tick" % (rec["bridge_ms"], step_ms))

    ver = drop_ball_and_verify(world, rec["t_snap"])
    print("  验证(600 tick):球末100tick均速=%.4f m/s(阈值 %.2f)" % (ver["avg_speed100"], STABLE_SPEED))
    print("    球底-块顶 = %.3f mm(容差 %.1f mm)| 球心水平漂移 %.4f m | 停在块上=%s"
          % (-ver["penetration"] * 1000, PEN_TOL * 1000, ver["drift"], ver["on_block"]))
    print("    有限性(无NaN/爆炸)=%s | 峰值|p|=%.3f | 总判定=%s" % (ver["finite"], ver["peak_abs"], "PASS" if ver["ok"] else "FAIL"))

    wall = time.perf_counter() - t_wall
    print("  单策略总耗时(含建世界): %.1f ms" % (wall * 1000))
    return {
        "label": label, "tick": tick, "timeout": timeout, "sleeping_at_trigger": sleeping_at,
        "lin": lin, "ang": ang, "first_slow": first_slow, "step_ms": step_ms,
        "ground_pen_mm": ground_pen * 1000, "wall_s": wall, **rec, **ver,
    }


def main():
    t_all = time.perf_counter()
    print("凝固桥(solidify bridge)可行性验证")
    print("rapier3d", r.__version__, "| dt = 1/60 | 24 轴对齐朝向已构造:", len(ROTS24))

    results = [run_strategy(*cfg) for cfg in STRATEGIES]

    # ---------------- 对比表 ----------------
    print("\n" + "=" * 64)
    print("## 触发策略对比")
    print("| 指标 | " + " | ".join(d["label"] for d in results) + " |")
    print("|---|" + "---|" * len(results))

    def row(name, f, fmt=str):
        return "| %s | %s |" % (name, " | ".join(fmt(f(d)) for d in results))

    print(row("触发等待 tick", lambda d: d["tick"]))
    print(row("触发等待(s)", lambda d: "%.2f" % (d["tick"] / 60.0)))
    print(row("触发时已睡眠", lambda d: d["sleeping_at_trigger"]))
    print(row("触发时线速度(m/s)", lambda d: "%.4f" % d["lin"]))
    print(row("触发时角速度(rad/s)", lambda d: "%.4f" % d["ang"]))
    print(row("snap 位移(mm)", lambda d: "%.3f" % (d["disp"] * 1000)))
    print(row("snap 位移竖直分量(mm)", lambda d: "%+.3f" % (d["disp_vec"][1] * 1000)))
    print(row("snap 角度误差(deg)", lambda d: "%.4f" % d["ang_err"]))
    print(row("snap 前姿态角陷入地面(mm)", lambda d: "%.3f" % (d["pre_pen"] * 1000)))
    print(row("snap 后块-地面穿深(mm)", lambda d: "%.3f" % d["ground_pen_mm"]))
    print(row("桥耗时(ms)", lambda d: "%.3f" % d["bridge_ms"]))
    print(row("验证:球均速(m/s)", lambda d: "%.4f" % d["avg_speed100"]))
    print(row("验证:球陷入块顶(mm)", lambda d: "%.3f" % (d["penetration"] * 1000)))
    print(row("验证:稳定+无穿模+有限", lambda d: "PASS" if d["ok"] else "FAIL"))

    # ---------------- 数据驱动结论 ----------------
    print("\n## 结论(数据驱动)")
    by_tick = sorted(results, key=lambda d: d["tick"])
    print("* 触发排序:%s(等待 tick:%s)" %
          (" -> ".join(d["label"] for d in by_tick),
           ", ".join(str(d["tick"]) for d in by_tick)))
    for d in results:
        print("* %s:snap 位移 %.3f mm(竖直 %+.3f mm),角度误差 %.4f deg,"
              "桥耗时 %.3f ms(=16.7ms 帧预算的 %.2f%%),验证 %s" %
              (d["label"], d["disp"] * 1000, d["disp_vec"][1] * 1000, d["ang_err"],
               d["bridge_ms"], d["bridge_ms"] / 16.7 * 100, "PASS" if d["ok"] else "FAIL"))
    print("* 总耗时:%.1f ms(远小于 3 分钟上限)" % ((time.perf_counter() - t_all) * 1000))


if __name__ == "__main__":
    main()
