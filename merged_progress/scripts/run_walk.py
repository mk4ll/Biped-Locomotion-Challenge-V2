"""Unified terrain-aware walking (merged): flat / incline / stairs.

Same offline->online->WBC pipeline as the rest of the repo, now terrain-aware:
the offline footstep planner places feet ON the surface (terrain.height /
footstep_x), the online DCM layer is unchanged, and the WBC friction cones are
expressed in the surface frame (terrain.normal). This replaces the bolted-on
incline of Stage 6 with the terrain abstraction (credit: terrain design adapted
from Kanellos' work).

  python scripts/run_walk.py --terrain flat
  python scripts/run_walk.py --terrain incline --angle 10
  python scripts/run_walk.py --terrain stairs
  python scripts/run_walk.py --terrain incline --angle 8 --viewer
"""
import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco

from src.utils.config import load_params
from src.sim.mujoco_env import make_robot_env
from src.control.walking_controller import WalkingController
from src.planning.walk_plan import WalkPlan
from src.planning.terrain import make_terrain


def build_on_terrain(params, terrain_name, angle_deg=8.0, stairs_kw=None, robot="g1"):
    """Create env+controller on a terrain and put the robot in a valid start pose.
    robot: 'g1' or 'talos' (robot-agnostic stack)."""
    if terrain_name == "incline":
        terrain = make_terrain("incline", angle=np.deg2rad(angle_deg))
    elif terrain_name == "stairs":
        terrain = make_terrain("stairs", **(stairs_kw or {}))
    else:
        terrain = make_terrain("flat")
    env, mcfg = make_robot_env(robot, terrain=terrain)
    ctrl = WalkingController(env, params, terrain=terrain, mcfg=mcfg)
    m, d = env.model, env.data

    if terrain_name == "incline":
        alpha = terrain.angle
        Rs = terrain.surface_R(0.0, 0.0)
        ctrl.ori_task.R_des = np.eye(3)                 # torso vertical vs gravity
        act_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
                     for a in range(m.nu)]
        for j in mcfg["ankle_pitch_joints"]:
            adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
            d.qpos[adr] -= alpha                        # pre-tilt feet flat on slope
            if j in act_names:
                ctrl.pos_task.q_nom[act_names.index(j)] -= alpha
        mujoco.mj_forward(m, d)
        nrm = Rs @ np.array([0., 0., 1.])
        corners = [c for foot in ctrl.contacts.stance for c in foot["corners"]]
        d.qpos[2] -= (min(d.site_xpos[c] @ nrm for c in corners) - 0.005)
        mujoco.mj_forward(m, d)
    # flat / stairs: crouch keyframe pose is already valid on the flat start ground
    return env, ctrl, terrain


def settle(env, ctrl, terrain, seconds=0.8):
    base = ctrl.base_id
    com_ref = env.data.subtree_com[base].copy()
    for _ in range(int(seconds / env.dt)):
        ctrl.com_task.a_ref = np.zeros(3)
        ctrl.com_task.p_ref = com_ref
        ctrl.com_task.v_ref = np.zeros(3)
        ctrl.contacts.set_support("double")
        tau_fb = ctrl.gc.compute(env.data,
                                 active_site_ids=ctrl.contacts.active_site_ids)[0]
        res = ctrl.wbc.solve(env.data, [ctrl.com_task, ctrl.ori_task, ctrl.pos_task],
                             ctrl.contacts.stance, R_surface=ctrl.R_surface,
                             fallback_tau=tau_fb)
        env.step(res["tau"])


def run(terrain_name="flat", angle_deg=8.0, viewer=False, robot="g1"):
    params = load_params()
    # Per-terrain gait tuning (step length must match the tread run for a
    # feet-together-per-tread stair gait; longer SS + more clearance to climb).
    stairs_kw = dict(rise=0.025, run=0.16, n_steps=6, x0=0.30)
    if terrain_name == "stairs":
        g = params["gait"]
        g["step_length"] = stairs_kw["run"]      # one tread per step (default timing works @0.16)
        g["swing_apex"] = 0.06                    # clear the riser
        g["n_steps"] = 18                         # approach + feet-together climb of all treads
    env, ctrl, terrain = build_on_terrain(params, terrain_name, angle_deg, stairs_kw, robot)
    settle(env, ctrl, terrain, 0.8)

    base = ctrl.base_id
    init_left = env.data.site_xpos[ctrl.left_site].copy()
    init_right = env.data.site_xpos[ctrl.right_site].copy()
    com0 = env.data.subtree_com[base].copy()
    plan = WalkPlan(params, init_left, init_right, com0,
                    com_height=params["gait"]["com_height"],
                    gravity=params["env"]["gravity"], terrain=terrain)
    x0 = com0[0]
    n = int(plan.duration / env.dt)
    log = {"x": [], "z": [], "bz": []}

    def loop(i):
        ctrl.step(plan, i * env.dt)
        c = env.data.subtree_com[base]
        log["x"].append(c[0]); log["z"].append(c[2]); log["bz"].append(env.data.qpos[2])
        # "fell" = base dropped well below the local surface height
        return env.data.qpos[2] - terrain.height(c[0], 0.0) < 0.45

    fell = False
    if viewer:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(env.model, env.data) as v:
            for i in range(n):
                if not v.is_running():
                    break
                if loop(i):
                    fell = True; break
                v.sync()
    else:
        for i in range(n):
            if loop(i):
                fell = True; break

    dist = env.data.subtree_com[base][0] - x0
    rise = env.data.subtree_com[base][2] - com0[2]
    label = terrain_name + (f" {angle_deg:.0f}deg" if terrain_name == "incline" else "")
    print(f"\n===== Merged terrain walk: {label} =====")
    print(f"fell             = {fell}")
    print(f"forward distance = {dist:.3f} m")
    print(f"height gain      = {rise*1000:.0f} mm")
    ok = (not fell) and dist > 0.3
    print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain", default="flat", choices=["flat", "incline", "stairs"])
    ap.add_argument("--angle", type=float, default=8.0)
    ap.add_argument("--robot", default="g1", choices=["g1", "talos"])
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()
    run(args.terrain, args.angle, args.viewer, args.robot)
