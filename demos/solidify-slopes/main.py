# -*- coding: utf-8 -*-
"""
斜坡(阶梯)/堆叠慢爬行下的 P2 凝固触发参数验证
================================================

REQ-001 决议 5 欠账(§9.5:"斜坡/堆叠慢爬行 demo 在 REQ-002 定稿前补")。
验证目标:P2 触发参数(线速度 <0.1 且角速度 <0.05 连续 20 tick,§4.3 /
demos/solidify-bridge)在"实体仍在有意运动但瞬时速度低于阈值"的慢爬行场景
下会不会误凝固。

单位说明(重要):P2 阈值 0.1/0.05 是 rapier 线速度单位 = m/s(dt=1/60,对应
每 tick 位移 0.1/60≈1.67mm)。任务书写的"0.01-0.09 m/tick"若按字面是
0.6-5.4 m/s,远高于阈值,不可能触发 P2;为让扫描区间覆盖"恰低于阈值"的语义,
主扫描取 0.01-0.09(m/s,与阈值同单位),另加一档字面 0.09 m/tick(=5.4 m/s)
对照,证明该档远离阈值、无触发。

测量与驱动约定(诚实条款):
  * 策略可见速度 = 每个 step() 之后、下一次驱动之前读到的 body.linvel
    (即凝固策略每 tick 真正会读到的信号);
  * "持续低速运动"用 P 控制器模拟:每 tick 结束若 vx < v_target,补冲量
    m*(v_target-vx) 把线速度钉回 v_target —— 这正是 Minecraft 实体每 tick
    被移动代码钉速度的物理对应(潜行 ~0.1 m/tick、水中缓行等);
  * 同时报告"净爬行速度 = 位移/时间",防止把粘性-滑移(stick-slip)的
    瞬时尖峰误当匀速:粗糙面上采样速度会塌缩到 ~0(每步内被静摩擦刹停),
    只会让 P2 更容易误触发,不会更宽松。

场景(rapier3d 0.36.0,demos/_API_NOTES.md 实测 API;凝固判定/snap/桥代码
复用 demos/solidify-bridge/main.py):
  A 平地匀速缓行:无摩擦面 + 线性阻尼 2.0,P 控制器钉速
    0.02/0.05/0.08 m/s 三档 + 字面单位对照档 5.4 m/s,各 600 tick;
  B 滚动球缓行:球 r=0.5,P 钉速 0.08 m/s(稳态滚动 ω=v/r≈0.16>0.05,
    检验角速度项是否保护滚动体;另单独计"起步瞬态"触发);
  C 惯性滑行:μ=0.01 低摩擦面,初速 0.09 m/s 无持续驱动(滑行中的掉落物),
    看 P2 与引擎睡眠兜底(can_sleep 默认 lin<0.05 持续 0.5s)谁先冻住它;
  D 阶梯下行:每级 1 格的下行阶梯(坡比 1:1 / 1:2 / 1:4),P 钉速 0.05 /
    0.09 m/s 爬行,1200 tick,统计级间爬行/翻落与 P2 触发的相互作用;
  E 堆叠:2 层 / 3 层,上层从间隙 0.6m 落到下层,600 tick;watch 模式记录
    每层 P2 触发时刻/触发时高度/触发前 20 tick 速度分布/触发后残余位移;
    bridge 模式在下层触发时真的执行凝固桥,观察上层是否被波及、落在凝固块
    上是否穿模、上层自己何时触发;
  F 对照:真静止立方体 600 tick:P2 必触发(正向验证桥仍工作)+ 掉球验证;
  G 探针:引擎睡眠阈值可否在 Python 绑定侧收紧(供修订建议可行性注脚)。

判据:误凝固 = 触发时刻实体仍在有意运动——受驱/滑行场景由构造保证(持续
驱动/无驱动但仍有净位移);堆叠场景量化为"触发后 100 tick 位移 resid100"
与"触发前 20 tick 位移 d20"。

运行:C:\\Users\\esp3j\\rep\\physics_minecraft\\.venv\\Scripts\\python.exe main.py
纯 stdout,无随机,物理确定,可重复。
"""

import collections
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
DT = 1.0 / 60.0
G = 9.81
CUBE_HALF = 0.5
GROUND_TOP = 0.0

# P2 触发参数(REQ-001 §4.3 / §9.5 决议 5)
LIN_T = 0.10
ANG_T = 0.05
NEED = 20

# 候选修订(P2'):在 P2 之上叠加位移门 —— 最近 20 tick 窗口位移 <= GATE_D20
# 阈值取 3mm:合法静止/落定实测 d20 <= 1.4mm(见 E/F),受驱爬行 d20 >= 6.3mm(见 A),
# 两侧各留 >=2x 裕度。
GATE_D20 = 0.003

BALL_VERIFY_TICKS = 600
BALL_GAP = 5.0
STABLE_SPEED = 0.05
PEN_TOL = 0.01
ON_BLOCK_R = 0.45


# ============ 通用工具(复用 demos/solidify-bridge/main.py) ============
def as_arr(v):
    try:
        return np.asarray(v, dtype=float)
    except Exception:
        return np.array([v.x, v.y, v.z], dtype=float)


def norm3(v):
    a = as_arr(v)
    return float(np.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]))


def build_24_rotations():
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
    return np.array(body.rotation.to_matrix(), dtype=float).reshape(3, 3)


def rel_angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def snap_rotation(R_pre):
    best, best_err = None, 1e18
    for C in ROTS24:
        err = rel_angle_deg(C, R_pre)
        if err < best_err:
            best, best_err = C, err
    return best, best_err


def mat_to_scaled_axis(R):
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if ang < 1e-9:
        return (0.0, 0.0, 0.0)
    if abs(math.pi - ang) < 1e-6:
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
    cells = [math.floor(c) for c in t]
    center = np.array([c + 0.5 for c in cells], dtype=float)
    return center, tuple(cells)


def solidify_ent(world, ent, voxels, friction=0.7):
    """凝固桥(demo C 流程):读态 -> snap 去旋转 -> snap 位置 -> 换静态体 -> 写体素。"""
    body = world.rigid_bodies[ent.handle]
    t_pre = as_arr(body.translation)
    R_pre = rotation_matrix(body)
    C, ang_err = snap_rotation(R_pre)
    t_snap, cell = snap_to_voxel_center(t_pre)
    world.remove_body(ent.handle)
    sa = mat_to_scaled_axis(C)
    bd = r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation(tuple(t_snap))
    if float(np.linalg.norm(sa)) > 1e-12:
        bd = bd.rotation(sa)
    world.add_body(bd, colliders=[r.Collider.cuboid(CUBE_HALF, CUBE_HALF, CUBE_HALF)
                                  .friction(friction).density(1.0)])
    voxels.add(cell)
    return {"t_pre": t_pre, "t_snap": t_snap, "cell": cell, "ang_err": ang_err,
            "disp": float(np.linalg.norm(t_snap - t_pre))}


# ============ P2 监视 ============
class P2:
    """按 tick 喂入策略可见的 (lin, ang, pos),模拟 P2 连续计数;达到 NEED 记一次
    触发并清零(watch 模式下等价于"若未真凝固,还会反复触发")。
    同时度量:
      - max_run_slow / max_run_lin:不随触发清零的"连续低于阈值"最长 run;
      - d20:每触发时刻回看最近 20 tick 的位置位移(候选"窗口位移门"判据的数据)。"""

    def __init__(self):
        self.consec = 0
        self.max_consec = 0          # 触发清零口径的最大连续(P2 重触发节奏)
        self.run_slow = 0
        self.max_run_slow = 0        # 连续(lin<0.1 且 ang<0.05)最长 run(不清零)
        self.run_lin = 0
        self.max_run_lin = 0         # 连续仅 lin<0.1 最长 run(不清零)
        self.n_triggers = 0
        self.trig_d20 = []           # 各触发时刻的 20-tick 窗口位移(m)
        self._hist = collections.deque(maxlen=NEED + 1)
        # 候选修订:P2' = P2 ∧ "最近 20 tick 位移 <= GATE_D20"(位移门)
        self.consec_g = 0
        self.n_gate_triggers = 0
        self.first_gate_tick = None

    def feed(self, tick, lin, ang, pos):
        slow = lin < LIN_T and ang < ANG_T
        self._hist.append(pos.copy())
        dwin = float(np.linalg.norm(pos - self._hist[0]))  # 最近 ≤20 tick 窗口位移
        self.run_slow = self.run_slow + 1 if slow else 0
        self.max_run_slow = max(self.max_run_slow, self.run_slow)
        self.run_lin = self.run_lin + 1 if lin < LIN_T else 0
        self.max_run_lin = max(self.max_run_lin, self.run_lin)
        self.consec = self.consec + 1 if slow else 0
        self.max_consec = max(self.max_consec, self.consec)
        if self.consec >= NEED:
            self.consec = 0
            self.n_triggers += 1
            self.trig_d20.append(dwin)
            ev = {"tick": tick, "lin": lin, "ang": ang, "d20": dwin,
                  "pos": None, "resid100": None}
        else:
            ev = None
        self.consec_g = self.consec_g + 1 if (slow and dwin <= GATE_D20) else 0
        if self.consec_g >= NEED:
            self.consec_g = 0
            self.n_gate_triggers += 1
            if self.first_gate_tick is None:
                self.first_gate_tick = tick
        return ev


class Ent:
    def __init__(self, name, handle, rest_y=None):
        self.name = name
        self.handle = handle
        self.p2 = P2()
        self.lin = []                # 策略可见线速度(step 后、驱动前)
        self.ang = []
        self.pos0 = None
        self.triggers = []           # P2.feed 事件(带 pos/d20/resid100/d_final)
        self.sleep_first = None      # 首次 is_sleeping 的 tick
        self.alive = True
        self.rest_y = rest_y         # 预期最终静止中心 y(用于悬空量化)
        self.final_pos = None
        self.bridge = None
        self.mass = None

    def slow_window_before(self, trig_idx, w=NEED):
        """第 trig_idx 次触发所用连续慢速窗口(20 个采样 tick)的 (mean, max) 线速度。
        自采样:tick t 采样存 lin[t-1];tick t 触发消耗采样 t-19..t -> 索引 [t-20:t]。"""
        t = self.triggers[trig_idx]["tick"]
        seg = self.lin[max(0, t - w):t]
        if not seg:
            return (0.0, 0.0)
        return (float(np.mean(seg)), float(np.max(seg)))


def make_world(mu_ground=0.7):
    w = r.PhysicsWorld(GRAVITY)
    w.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED).translation((0.0, -0.5, 0.0)),
               colliders=[r.Collider.cuboid(50.0, 0.5, 50.0).friction(mu_ground)])
    return w


def add_cube(world, name, pos, mu=0.2, v=(0.0, 0.0, 0.0), lin_damp=0.0,
             ang_damp=0.0, can_sleep=True, rest_y=None):
    h = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(tuple(pos)).linvel(tuple(v))
        .linear_damping(lin_damp).angular_damping(ang_damp)
        .can_sleep(can_sleep),
        colliders=[r.Collider.cuboid(CUBE_HALF, CUBE_HALF, CUBE_HALF)
                   .friction(mu).restitution(0.0).density(1.0)])
    e = Ent(name, h, rest_y=rest_y)
    e.mass = float(world.rigid_bodies[h].mass)
    return e


def add_ball(world, name, pos, mu=0.5, v=(0.0, 0.0, 0.0), ang_damp=0.0):
    h = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation(tuple(pos)).linvel(tuple(v))
        .angular_damping(ang_damp).can_sleep(True),
        colliders=[r.Collider.ball(0.5).friction(mu).restitution(0.0).density(1.0)])
    e = Ent(name, h)
    e.mass = float(world.rigid_bodies[h].mass)
    return e


def p_drive_factory(world, ents, v_target):
    """P 控制器:每 tick 若 vx < v_target,补冲量把线速度钉回 v_target。
    (Minecraft 实体每 tick 被移动代码钉速度的物理对应。)"""
    def drive(t, alive_handles):
        for h in alive_handles:
            b = world.rigid_bodies[h]
            vx = float(b.linvel.x)
            if vx < v_target:
                b.apply_impulse((b.mass * (v_target - vx), 0.0, 0.0), True)
    return drive


def run_sim(world, ents, ticks, drive=None, on_trigger=None):
    """通用循环:step -> 采样每个活实体(策略可见信号:step 后、驱动前)喂 P2 ->
    外部驱动 drive(t, alive_handles)。on_trigger(ent, ev) 可在触发时执行凝固桥
    (置 ent.alive=False 停止采样)。触发后 +100 tick 记录残余位移 resid100。"""
    for t in range(1, ticks + 1):
        world.step()
        for e in ents:
            if not e.alive:
                continue
            b = world.rigid_bodies[e.handle]
            lin, ang = norm3(b.linvel), norm3(b.angvel)
            pos = as_arr(b.translation)
            if e.pos0 is None:
                e.pos0 = pos.copy()
            e.lin.append(lin)
            e.ang.append(ang)
            if b.is_sleeping and e.sleep_first is None:
                e.sleep_first = t
            ev = e.p2.feed(t, lin, ang, pos)
            if ev is not None:
                ev["pos"] = pos.copy()
                e.triggers.append(ev)
                if on_trigger is not None:
                    on_trigger(e, ev)
            for ev2 in e.triggers:
                if ev2["resid100"] is None and t == ev2["tick"] + 100:
                    ev2["resid100"] = float(np.linalg.norm(pos - ev2["pos"]))
        if drive is not None:
            drive(t, [e.handle for e in ents if e.alive])
        for e in ents:
            if e.alive:
                e.final_pos = as_arr(world.rigid_bodies[e.handle].translation)
    # 每个触发时刻到"最终采样位置"的距离(= 若当时真凝固,实体还会被 snap 吞掉多少运动)
    for e in ents:
        if e.alive and e.final_pos is not None:
            for ev in e.triggers:
                if ev["pos"] is not None:
                    ev["d_final"] = float(np.linalg.norm(ev["pos"] - e.final_pos))
    return ents


# ============ 统计 ============
def vstats(vals):
    a = np.asarray(vals, dtype=float)
    if a.size == 0:
        return dict(n=0, mean=0.0, min=0.0, max=0.0, p50=0.0, p90=0.0, p99=0.0)
    return dict(n=int(a.size), mean=float(a.mean()), min=float(a.min()), max=float(a.max()),
                p50=float(np.percentile(a, 50)), p90=float(np.percentile(a, 90)),
                p99=float(np.percentile(a, 99)))


def trig_str(e, with_win=False):
    if not e.triggers:
        return "无触发"
    ev = e.triggers[0]
    s = "@%d(采样lin=%.4f 采样ang=%.4f d20=%.4fm" % (ev["tick"], ev["lin"], ev["ang"], ev["d20"])
    if with_win:
        m, mx = e.slow_window_before(0)
        s += " 触发前20tick均速=%.6f max=%.6f" % (m, mx)
    if ev["pos"] is not None:
        s += " y=%.4f" % ev["pos"][1]
    s += ",resid100=%s" % ("%.6f" % ev["resid100"] if ev["resid100"] is not None else "-")
    s += ",d_final=%s)" % ("%.4f" % ev["d_final"] if ev.get("d_final") is not None else "-")
    if e.p2.n_triggers > 1:
        s += " x%d" % e.p2.n_triggers
    return s


# ============ 场景 A:平地匀速缓行 ============
def scenario_creep_flat(targets=(0.02, 0.05, 0.08, 5.4), ticks=600):
    print("\n" + "=" * 72)
    print("场景 A:平地匀速缓行(无摩擦面+线性阻尼 2.0,P 控制器钉速,%d tick)" % ticks)
    print("  P2:lin<%.2f 且 ang<%.2f 连续 %d tick;采样口径=step 后、驱动前" %
          (LIN_T, ANG_T, NEED))
    print("=" * 72)
    rows = []
    for tgt in targets:
        w = make_world(0.0)
        e = add_cube(w, "cube", (0.5, 0.5, 0.5), mu=0.0, lin_damp=2.0)
        run_sim(w, [e], ticks, drive=p_drive_factory(w, [e], tgt))
        st = vstats(e.lin[100:])
        sa = vstats(e.ang[100:])
        net = float(np.linalg.norm(e.final_pos - e.pos0)) / (ticks * DT)
        print("目标 %.3f m/s:采样 lin p50=%.4f p99=%.4f(%d-%.4f),ang p99=%.5f,"
              "净爬行速度=%.4f m/s(位移 %.4f m)" %
              (tgt, st["p50"], st["p99"], st["min"], st["max"], sa["p99"],
               net, float(np.linalg.norm(e.final_pos - e.pos0))))
        print("  连续(lin<0.1∧ang<0.05)最长=%d tick(是否达 %d:%s),连续(仅lin)最长=%d;"
              "P2 触发 %d 次%s | P2'(位移门 %.1fmm)触发 %d 次%s" %
              (e.p2.max_run_slow, NEED, "是" if e.p2.max_run_slow >= NEED else "否",
               e.p2.max_run_lin, e.p2.n_triggers,
               " -> 全部为误凝固(驱动持续、净速度 %.4f m/s)" % net if e.p2.n_triggers else "",
               GATE_D20 * 1000, e.p2.n_gate_triggers,
               "" if e.p2.n_gate_triggers == 0 else "@tick %s" % e.p2.first_gate_tick))
        print("  首次触发:%s" % trig_str(e))
        rows.append((tgt, st, net, e))
    print("| 目标 m/s | 采样lin p50 | 净爬行 m/s | 最长连(lin∧ang) | 首触发tick | 首触发d20(m) | resid100(m) | d_final(m) | P2触发次数 | P2'(3mm门)次数 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for tgt, st, net, e in rows:
        ev = e.triggers[0] if e.triggers else None
        print("| %.2f | %.4f | %.4f | %d | %s | %s | %s | %s | %d | %d |" % (
            tgt, st["p50"], net, e.p2.max_run_slow,
            ev["tick"] if ev else "-",
            "%.4f" % ev["d20"] if ev else "-",
            ("%.4f" % ev["resid100"]) if ev and ev["resid100"] is not None else "-",
            ("%.4f" % ev["d_final"]) if ev and ev.get("d_final") is not None else "-",
            e.p2.n_triggers, e.p2.n_gate_triggers))
    return rows


# ============ 场景 B:滚动球缓行 ============
def scenario_creep_ball(target=0.08, mu=0.5, ticks=600):
    print("\n" + "=" * 72)
    print("场景 B:滚动球缓行(r=0.5,μ=%.1f,P 钉速 %.3f m/s,%d tick)"
          "——角速度项是否保护滚动体" % (mu, target, ticks))
    print("=" * 72)
    w = make_world(mu)
    e = add_ball(w, "ball", (0.5, 0.5, 0.0), mu=mu)
    run_sim(w, [e], ticks, drive=p_drive_factory(w, [e], target))
    st = vstats(e.lin[100:])
    sa = vstats(e.ang[100:])
    net = float(np.linalg.norm(e.final_pos - e.pos0)) / (ticks * DT)
    startup = [ev for ev in e.triggers if ev["tick"] <= 30]
    steady = [ev for ev in e.triggers if ev["tick"] > 30]
    print("采样 lin p50=%.4f(ang p50=%.4f≈v/r=%.4f),稳态滚动 ang p99=%.4f;"
          "净爬行速度=%.4f m/s" % (st["p50"], sa["p50"], target * 2, sa["p99"], net))
    print("连续(lin∧ang)最长=%d,连续(仅lin)最长=%d;P2 触发 %d 次(其中起步瞬态≤30tick %d 次,"
          "稳态滚动期 %d 次);P2'(位移门)触发 %d 次" %
          (e.p2.max_run_slow, e.p2.max_run_lin, e.p2.n_triggers, len(startup), len(steady),
           e.p2.n_gate_triggers))
    print("  首次触发:%s" % trig_str(e, with_win=True))
    return e


# ============ 场景 C:惯性滑行 ============
def scenario_glide(v0=0.09, mu=0.01, ticks=600):
    print("\n" + "=" * 72)
    print("场景 C:低摩擦惯性滑行(μ=%.2f,初速 %.3f m/s,无持续驱动,%d tick)"
          "——滑行中的掉落物:P2 与睡眠兜底谁先冻住它" % (mu, v0, ticks))
    print("=" * 72)
    w = make_world(mu)
    e = add_cube(w, "cube", (0.5, 0.5, 0.0), mu=mu, v=(v0, 0.0, 0.0))
    b0 = w.rigid_bodies[e.handle]
    thr_lin = b0.activation.linear_threshold
    thr_ang = b0.activation.angular_threshold
    tts = b0.activation.time_until_sleep
    run_sim(w, [e], ticks)
    ideal_stop = v0 * v0 / (2.0 * mu * G)          # 无冻结应滑行的总距离
    traveled = float(e.final_pos[0] - e.pos0[0])
    print("引擎睡眠阈值(实测读出):linear=%.4f angular=%.2f time_until_sleep=%.2f s"
          % (thr_lin, thr_ang, tts))
    print("连续(lin∧ang)最长=%d,连续(仅lin)最长=%d" %
          (e.p2.max_run_slow, e.p2.max_run_lin))
    print("P2 首次触发:%s" % trig_str(e))
    print("P2'(位移门 %.1fmm)首次触发@tick %s(共 %d 次):门挡住滑行,直到窗口位移 < %.1fmm 才允许凝固"
          % (GATE_D20 * 1000, e.p2.first_gate_tick, e.p2.n_gate_triggers, GATE_D20 * 1000))
    print("引擎 is_sleeping 首次@tick %s;watch 模式不真凝固,实体靠摩擦自然滑停:"
          "库仑模型应滑 %.4f m,实测 %.4f m(差额为求解器数值耗散);"
          "若按 P2 在 tick20 真凝固,滑行中途即被冻住,距最终静止位还差 d_final=%.4f m"
          % (e.sleep_first, ideal_stop, traveled,
             e.triggers[0].get("d_final") if e.triggers else float("nan")))
    return e


# ============ 场景 D:阶梯下行 ============
def make_stairs(R, S, mu):
    """每级 1 格高、水平跑 R 格的下行阶梯(x 增大方向下降),建于地面之上。"""
    w = make_world(mu)
    for i in range(S * R):
        top = S - 1 - i // R
        for lvl in range(top + 1):
            w.add_body(r.RigidBodyBuilder(r.RigidBodyType.FIXED)
                       .translation((i + 0.5, lvl + 0.5, 0.5)),
                       colliders=[r.Collider.cuboid(CUBE_HALF, CUBE_HALF, CUBE_HALF)
                                  .friction(mu)])
    return w


def scenario_stairs(R, target, mu=0.2, S=8, ticks=1200):
    w = make_stairs(R, S, mu)
    x0 = float(R) - 0.55                            # 质心越过台阶缘 x=R 即翻落
    e = add_cube(w, "cube", (x0, S + 0.5, 0.5), mu=mu)
    run_sim(w, [e], ticks, drive=p_drive_factory(w, [e], target))
    la = np.asarray(e.lin)
    falls = np.where(la > 0.5)[0]
    first_fall = int(falls[0]) + 1 if falls.size else None   # 1-based tick
    st = vstats(la)
    net = float(np.linalg.norm(e.final_pos - e.pos0)) / (ticks * DT)
    ev0 = e.triggers[0] if e.triggers else None
    print("坡比 1:%d,钉速目标 %.3f m/s:采样 lin p50=%.4f p99=%.4f max=%.3f,"
          "净速度=%.4f m/s;翻落(lin>0.5)@tick %s;连续(lin∧ang)最长=%d,连续(仅lin)最长=%d;"
          "P2 触发 %d 次;P2'(位移门)触发 %d 次%s" %
          (R, target, st["p50"], st["p99"], st["max"], net,
           first_fall if first_fall else "无", e.p2.max_run_slow, e.p2.max_run_lin,
           e.p2.n_triggers, e.p2.n_gate_triggers,
           ("(首次@%s)" % e.p2.first_gate_tick) if e.p2.n_gate_triggers else ""))
    print("  首次触发:%s;触发时 d20 区间 %s" %
          (trig_str(e, with_win=True),
           "[%.4f, %.4f]" % (min(e.p2.trig_d20), max(e.p2.trig_d20)) if e.p2.trig_d20 else "-"))
    return e, first_fall, ev0


def scenario_stairs_all():
    print("\n" + "=" * 72)
    print("场景 D:阶梯下行慢爬行(每级 1 格;坡比 1:1/1:2/1:4;钉速 0.05 与 0.09 m/s;1200 tick)")
    print("=" * 72)
    out = {}
    for R in (1, 2, 4):
        for tgt in (0.05, 0.09):
            out[(R, tgt)] = scenario_stairs(R, tgt)
    print("\n阶梯小结(P2 若在爬行中冻结 = 误凝固:驱动持续、实体还要翻落):")
    for (R, tgt), (e, ff, ev0) in out.items():
        t0 = ev0["tick"] if ev0 else None
        pre = (t0 is not None and (ff is None or t0 < ff))
        print("  1:%d @%.2f:首触发tick=%s(%s),触发%d次,最小d20=%s m,"
              "resid100=%s m,净速度=%.4f m/s -> 误凝固%s" %
              (R, tgt, t0,
               "早于翻落@%d" % ff if (pre and ff) else ("无翻落仍触发" if pre else "晚于翻落"),
               e.p2.n_triggers,
               "%.4f" % min(e.p2.trig_d20) if e.p2.trig_d20 else "-",
               ("%.4f" % ev0["resid100"]) if ev0 and ev0["resid100"] is not None else "-",
               float(np.linalg.norm(e.final_pos - e.pos0)) / (1200 * DT),
               "确认" if pre else "无(无触发)"))
    return out


# ============ 场景 E:堆叠 ============
def scenario_stack(n=2, gap=0.6, ticks=600, do_bridge=False):
    tag = "bridge(下层触发即真执行桥)" if do_bridge else "watch(只计数)"
    print("\n" + "=" * 72)
    print("场景 E:堆叠 %d 层,上层起始间隙 %.2f m,%s,%d tick" % (n, gap, tag, ticks))
    print("=" * 72)
    w = make_world(0.7)
    ents = []
    ents.append(add_cube(w, "L0(底)", (0.5, 0.5, 0.5), mu=0.7, rest_y=0.5))
    for j in range(1, n):
        ents.append(add_cube(w, "L%d" % j, (0.5, 1.5 + (j - 1) * 1.0 + gap, 0.5),
                             mu=0.7, rest_y=0.5 + j))
    voxels = set()

    def on_trigger(e, ev):
        if do_bridge and e.name.startswith("L0"):
            e.bridge = solidify_ent(w, e, voxels)
            e.alive = False          # 刚体已删除,停止采样

    run_sim(w, ents, ticks, on_trigger=on_trigger)
    for e in ents:
        st_all = vstats(e.lin)
        st_steady = vstats(e.lin[200:])
        print("%s:lin 全程 p50=%.6f p99=%.6f max=%.4f;稳态段(>200tick) p50=%.6f;"
              "连续(lin∧ang)最长=%d;首触发 %s;P2'(位移门)首触发@tick %s 共%d次;"
              "终点 y=%.4f(预期 %.2f)" %
              (e.name, st_all["p50"], st_all["p99"], st_all["max"], st_steady["p50"],
               e.p2.max_run_slow, trig_str(e, with_win=True), e.p2.first_gate_tick,
               e.p2.n_gate_triggers, e.final_pos[1], e.rest_y))
        if e.triggers:
            ev = e.triggers[0]
            air = ev["pos"][1] - e.rest_y
            print("   首触发@%d:触发时中心高出预期静止位 %+.4f m(>0=仍在沉降);触发后残余位移 resid100=%s" %
                  (ev["tick"], air,
                   "%.6f" % ev["resid100"] if ev["resid100"] is not None else "(已凝固)"))
        if e.bridge:
            print("   桥:snap 位移 %.1f mm,角度误差 %.4f deg -> 体素格 %s" %
                  (e.bridge["disp"] * 1000, e.bridge["ang_err"], e.bridge["cell"]))
    if do_bridge:
        top = 1.0                                    # L0 凝固块顶(格 (0,0,0) 顶面)
        up = ents[1]
        print("   上层落定检验:终点中心 y=%.4f(理想 1.5),底面-凝固块顶=%.4f m(穿模容差 %.2f)"
              % (up.final_pos[1], up.final_pos[1] - CUBE_HALF - top, PEN_TOL))
        print("   体素数组=%s" % sorted(voxels))
    return ents


# ============ 场景 F:对照 ============
def drop_ball_and_verify(world, block_center):
    """demo C 的验证:块顶上方 5 格掉球,600 tick 查稳定/穿模/有限性。"""
    top = block_center[1] + CUBE_HALF
    bx, bz = block_center[0], block_center[2]
    handle = world.add_body(
        r.RigidBodyBuilder(r.RigidBodyType.DYNAMIC)
        .translation((bx, top + BALL_GAP + CUBE_HALF, bz))
        .linvel((0.05, 0.0, 0.0))
        .angular_damping(2.0)
        .can_sleep(True),
        colliders=[r.Collider.ball(CUBE_HALF).friction(0.7).restitution(0.05).density(1.0)])
    ys, speeds = [], []
    finite = True
    for _ in range(BALL_VERIFY_TICKS):
        world.step()
        b = world.rigid_bodies[handle]
        p = as_arr(b.translation)
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(as_arr(b.linvel)))):
            finite = False
            break
        ys.append(float(p[1]))
        speeds.append(norm3(b.linvel))
    n = min(100, len(ys))
    avg_y = sum(ys[-n:]) / n
    avg_speed = sum(speeds[-n:]) / n
    penetration = top - (avg_y - CUBE_HALF)
    p_end = as_arr(world.rigid_bodies[handle].translation)
    drift = math.hypot(float(p_end[0]) - bx, float(p_end[2]) - bz)
    ok = avg_speed < STABLE_SPEED and drift <= ON_BLOCK_R and finite and penetration <= PEN_TOL
    return {"avg_speed100": avg_speed, "penetration": penetration, "finite": finite,
            "drift": drift, "ok": ok}


def scenario_control(ticks=600):
    print("\n" + "=" * 72)
    print("场景 F:对照——真静止实体 %d tick 必凝固 + 桥正向验证" % ticks)
    print("=" * 72)
    w = make_world(0.7)
    e = add_cube(w, "cube", (0.5, 0.5, 0.5), mu=0.7, rest_y=0.5)
    voxels = set()

    def on_trigger(ent, ev):
        ent.bridge = solidify_ent(w, ent, voxels)
        ent.alive = False

    run_sim(w, [e], ticks, on_trigger=on_trigger)
    print("真静止:P2 首触发 %s(期望 tick=20),连续(lin∧ang)最长=%d;"
          "P2'(位移门)首触发@tick %s(期望≈20,正向验证不受门影响)" %
          (trig_str(e), e.p2.max_run_slow, e.p2.first_gate_tick))
    print("桥:snap 位移 %.3f mm,角度误差 %.4f deg,体素=%s" %
          (e.bridge["disp"] * 1000, e.bridge["ang_err"], sorted(voxels)))
    ver = drop_ball_and_verify(w, e.bridge["t_snap"])
    print("掉球验证 600 tick:末100均速 %.4f m/s,陷入 %.3f mm,漂移 %.4f m,有限=%s -> %s" %
          (ver["avg_speed100"], ver["penetration"] * 1000, ver["drift"],
           ver["finite"], "PASS" if ver["ok"] else "FAIL"))
    return e, ver


# ============ 场景 G:睡眠阈值可否收紧(修订建议可行性探针) ============
def scenario_sleep_probe(v0=0.09, mu=0.01, ticks=600):
    print("\n" + "=" * 72)
    print("场景 G:探针——Python 绑定侧能否把引擎睡眠 linear_threshold 从 0.05 收紧到 0.01")
    print("=" * 72)
    try:
        w = make_world(mu)
        e = add_cube(w, "cube", (0.5, 0.5, 0.0), mu=mu, v=(v0, 0.0, 0.0))
        act = w.rigid_bodies[e.handle].activation
        act.linear_threshold = 0.01            # 是否抛异常?
        got = w.rigid_bodies[e.handle].activation.linear_threshold
        run_sim(w, [e], ticks)
        effect = "生效" if abs(got - 0.01) < 1e-6 else "被静默忽略(回读仍 %.4f)" % got
        print("赋值未抛异常;回读 linear_threshold=%.4f -> %s;"
              "is_sleeping 首次@tick %s(场景 C 默认 0.05 时同为 31,行为未变)"
              % (got, effect, e.sleep_first))
        return ("writable" if abs(got - 0.01) < 1e-6 else "silently-ignored", e.sleep_first)
    except Exception as ex:
        print("写入失败:%s: %s -> 收紧睡眠阈值需走 Rust 侧配置" %
              (type(ex).__name__, ex))
        return ("not writable", None)


# ============ 场景 H:睡眠兜底在"持续驱动"下是否兜得住 ============
def scenario_sleep_fallback(v_target=0.02, ticks=600):
    print("\n" + "=" * 72)
    print("场景 H:睡眠兜底探针——0.02 m/s 缓爬(< 引擎睡眠线阈值 0.05)会不会被兜底冻住?"
          "(决议 5 的兜底路径)")
    print("=" * 72)
    for wake in (True, False):
        w = make_world(0.0)
        e = add_cube(w, "cube", (0.5, 0.5, 0.5), mu=0.0, lin_damp=2.0)
        first_sleep = None
        for t in range(1, ticks + 1):
            w.step()
            b = w.rigid_bodies[e.handle]
            vx = float(b.linvel.x)
            if vx < v_target:
                b.apply_impulse((b.mass * (v_target - vx), 0.0, 0.0), wake)
            if b.is_sleeping and first_sleep is None:
                first_sleep = t
        print("驱动冲量 wake_up=%s:is_sleeping 首次@tick %s -> %s" %
              (wake, first_sleep,
               "睡眠兜底失效:每 tick 驱动都把它唤醒,P2 若触发无兜底可拦"
               if first_sleep is None else "兜底在 tick %d 生效" % first_sleep))


# ============ 主流程 ============
def main():
    t_all = time.perf_counter()
    print("斜坡/堆叠慢爬行 P2 触发参数验证(REQ-001 决议 5 欠账)")
    print("rapier3d %s | dt=1/60 | P2: lin<%.2f 且 ang<%.2f 连续 %d tick | 无随机,可重复"
          % (r.__version__, LIN_T, ANG_T, NEED))

    scenario_creep_flat()
    scenario_creep_ball()
    scenario_glide()
    scenario_stairs_all()
    scenario_stack(2, gap=0.6, do_bridge=False)
    scenario_stack(3, gap=0.6, do_bridge=False)
    scenario_stack(2, gap=0.6, do_bridge=True)
    scenario_control()
    scenario_sleep_probe()
    scenario_sleep_fallback()

    print("\n总耗时 %.1f s" % (time.perf_counter() - t_all))


if __name__ == "__main__":
    main()
