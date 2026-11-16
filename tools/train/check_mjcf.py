#!/usr/bin/env python3
"""REQ-012 E02a §4:核对导出的 MJCF 与本项目身体同构。

用法:python check_mjcf.py <mjcf> <report.json> <pose.json>
逐项打印,任何一项(1–4)不过以非零退出;第 5 项只报数。
四元数:转储是 glam x y z w,MuJoCo 是 w x y z,脚本内部转换。
"""
import json
import sys

import mujoco
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

FAILED = False


def check(ok, msg):
    global FAILED
    print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
    if not ok:
        FAILED = True


def rel_err(got, want):
    return abs(got - want) / abs(want) if want != 0 else abs(got - want)


def xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


def quat_angle(q1_wxyz, q2_wxyz):
    a = np.asarray(q1_wxyz, dtype=float)
    b = np.asarray(q2_wxyz, dtype=float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    dot = min(1.0, abs(float(np.dot(a, b))))
    return 2.0 * np.arccos(dot)


def body_id(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)


def joint_id(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)


def actuator_id(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def item1_bodies(m, links):
    print("项1:body 数与名字顺序")
    check(m.nbody - 1 == len(links), f"nbody-1={m.nbody - 1} == 连杆数 {len(links)}")
    for i, lk in enumerate(links):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i + 1)
        check(name == lk["name"], f"body[{i + 1}] 名字 {name!r} == {lk['name']!r}")


def item2_mass(m, links):
    print("项2:质量/质心/惯量")
    for i, lk in enumerate(links):
        bi = i + 1
        mass = float(m.body_mass[bi])
        check(rel_err(mass, lk["mass"]) < 1e-5,
              f"{lk['name']} mass={mass:.6g} 报告 {lk['mass']:.6g} 相对误差 {rel_err(mass, lk['mass']):.2e}")
        ipos = np.asarray(m.body_ipos[bi], dtype=float)
        com = np.asarray(lk["com"], dtype=float)
        dmax = float(np.max(np.abs(ipos - com)))
        check(dmax < 1e-6, f"{lk['name']} ipos 与 com 最大差 {dmax:.2e}")
        principal = np.sort(np.asarray(m.body_inertia[bi], dtype=float))
        eig = np.sort(np.linalg.eigvalsh(np.asarray(lk["inertia"], dtype=float)))
        ok = True
        worst = 0.0
        for g, w in zip(principal, eig):
            e = abs(g - w)
            r = e / abs(w) if w != 0 else e
            worst = max(worst, r)
            if not (r < 1e-4 or e < 1e-9):
                ok = False
        check(ok, f"{lk['name']} 主惯量排序 vs 报告特征值排序 最大相对误差 {worst:.2e}")


def item3_dofs(m, dofs):
    print("项3:自由度/执行器")
    type_map = {"hinge": int(mujoco.mjtJoint.mjJNT_HINGE),
                "slide": int(mujoco.mjtJoint.mjJNT_SLIDE)}
    for d in dofs:
        jid = joint_id(m, d["joint"])
        check(jid >= 0, f"关节 {d['joint']} 存在")
        if jid < 0:
            continue
        check(int(m.jnt_type[jid]) == type_map[d["type"]],
              f"{d['joint']} 类型 {int(m.jnt_type[jid])} == {d['type']}({type_map[d['type']]})")
        axis = np.asarray(m.jnt_axis[jid], dtype=float)
        dmax = float(np.max(np.abs(axis - np.asarray(d["axis"], dtype=float))))
        check(dmax < 1e-6, f"{d['joint']} axis 最大差 {dmax:.2e}")
        if d["lower"] is not None and d["upper"] is not None:
            rng = np.asarray(m.jnt_range[jid], dtype=float)
            dmax = float(np.max(np.abs(rng - np.array([d["lower"], d["upper"]]))))
            check(dmax < 1e-6, f"{d['joint']} range 最大差 {dmax:.2e}")
        aid = actuator_id(m, d["child"] + "_act")
        check(aid >= 0, f"执行器 {d['child']}_act 存在")
        if aid < 0:
            continue
        if d["max_torque"] is not None:
            fr = np.asarray(m.actuator_forcerange[aid], dtype=float)
            tau = d["max_torque"]
            dmax = float(np.max(np.abs(fr - np.array([-tau, tau]))))
            check(dmax < 1e-6, f"{d['child']}_act forcerange={fr.tolist()} == ±{tau} 差 {dmax:.2e}")
        kp, kd = d["kp"], d["kd"]
        gain0 = float(m.actuator_gainprm[aid][0])
        bias1 = float(m.actuator_biasprm[aid][1])
        bias2 = float(m.actuator_biasprm[aid][2])
        check(rel_err(gain0, kp) < 1e-6 if kp else abs(gain0) < 1e-6,
              f"{d['child']}_act gainprm[0]={gain0:.6g} == kp {kp:.6g}")
        check(rel_err(bias1, -kp) < 1e-6 if kp else abs(bias1) < 1e-6,
              f"{d['child']}_act biasprm[1]={bias1:.6g} == -kp {-kp:.6g}")
        check(rel_err(bias2, -kd) < 1e-6 if kd else abs(bias2) < 1e-6,
              f"{d['child']}_act biasprm[2]={bias2:.6g} == -kd {-kd:.6g}")


def item4_kinematics(m, links, dofs, pose):
    print("项4:运动学同构(每帧 mj_forward 比对世界位姿)")
    d = mujoco.MjData(m)
    jids = [joint_id(m, dd["joint"]) for dd in dofs]
    qadr = [int(m.jnt_qposadr[j]) for j in jids]
    for fr in pose["frames"]:
        mujoco.mj_resetData(m, d)
        root_pos = np.asarray(fr["root_pos"], dtype=float)
        root_quat = xyzw_to_wxyz(fr["root_quat"])
        d.qpos[0:3] = root_pos
        d.qpos[3:7] = root_quat
        for k, adr in enumerate(qadr):
            d.qpos[adr] = fr["joint_pos"][k]
        mujoco.mj_forward(m, d)
        worst_pos = 0.0
        worst_ang = 0.0
        for i, lk in enumerate(links):
            bi = i + 1
            xpos = np.asarray(d.xpos[bi], dtype=float)
            want = np.asarray(fr["links"][i]["pos"], dtype=float)
            worst_pos = max(worst_pos, float(np.max(np.abs(xpos - want))))
            ang = quat_angle(d.xquat[bi], xyzw_to_wxyz(fr["links"][i]["quat"]))
            worst_ang = max(worst_ang, float(ang))
        check(worst_pos < 1e-4, f"帧 step={fr['step']} 连杆位置最大差 {worst_pos:.2e} m")
        check(worst_ang < 1e-3, f"帧 step={fr['step']} 连杆朝向最大夹角 {worst_ang:.2e} rad")


def item5_stand(m, links, pose):
    print("项5:站立高度(只报数,不设阈值)")
    d = mujoco.MjData(m)
    keyid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if keyid < 0:
        print("  无 stand 关键帧,跳过")
        return
    mujoco.mj_resetDataKeyframe(m, d, keyid)
    root_bi = body_id(m, links[0]["name"])
    dt = float(m.opt.timestep)
    nsteps = int(round(5.0 / dt))
    last_n = int(round(1.0 / dt))
    heights = []
    for _ in range(nsteps):
        mujoco.mj_step(m, d)
        heights.append(float(d.xpos[root_bi][1]))
    tail = np.asarray(heights[-last_n:], dtype=float)
    mean = float(np.mean(tail))
    std = float(np.std(tail))
    game_abs = pose["stand_root_height"] + pose["ground_top_y"]
    print(f"  MuJoCo 最后 1 秒机身高度(y,地面=0):均值 {mean:.6f} m,标准差 {std:.6f} m")
    print(f"  游戏 stand_root_height={pose['stand_root_height']:.6f} m,ground_top_y={pose['ground_top_y']:.6f} m")
    print(f"  游戏根绝对高度 stand_root_height+ground_top_y={game_abs:.6f} m(地面基准 y={pose['ground_top_y']})")
    print(f"  同基准(离地高度)差:MuJoCo {mean:.6f} - 游戏 stand_root_height {pose['stand_root_height']:.6f} = {mean - pose['stand_root_height']:.6f} m")


def main():
    if len(sys.argv) != 4:
        print("用法:python check_mjcf.py <mjcf> <report.json> <pose.json>")
        sys.exit(2)
    xml, report_path, pose_path = sys.argv[1:4]
    m = mujoco.MjModel.from_xml_path(xml)
    report = json.load(open(report_path, encoding="utf-8"))
    pose = json.load(open(pose_path, encoding="utf-8"))
    links = report["links"]
    dofs = report["dofs"]
    print(f"模型 {xml}:nbody={m.nbody} njnt={m.njnt} nu={m.nu} nq={m.nq}")
    item1_bodies(m, links)
    item2_mass(m, links)
    item3_dofs(m, dofs)
    item4_kinematics(m, links, dofs, pose)
    item5_stand(m, links, pose)
    print("==== 结论:", "有 FAIL" if FAILED else "项1–4 全部 PASS", "====")
    sys.exit(1 if FAILED else 0)


if __name__ == "__main__":
    main()