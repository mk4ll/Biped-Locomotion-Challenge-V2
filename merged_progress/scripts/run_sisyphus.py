"""FUN task B — Sisyphus: the robot pushes a boulder up an incline.

A free, heavy sphere ("boulder") sits on the slope just ahead of the robot. The
robot walks uphill (terrain-aware incline gait) and shoves the boulder up the
slope by body contact. If it ever stops pushing, gravity rolls the rock back
down -- the Sisyphus drama. We track how far up the rock is pushed.

  python scripts/run_sisyphus.py --angle 6
  python scripts/run_sisyphus.py --angle 6 --viewer
  python scripts/run_sisyphus.py --angle 8 --mass 2.5 --robot talos
"""
import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.utils.config import load_params
from src.planning.walk_plan import WalkPlan

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_walk import build_on_terrain, settle


def boulder_decorator(angle_rad, x0, radius, mass):
    """Add a free 'boulder' (box) resting on the incline at x0 (uphill = +x).

    A box (not a sphere) so it does not roll freely down the slope -- with high
    friction it stays where it is pushed (tan(angle) < mu), and the robot shoves
    it up by body/shin contact. Lower + a bit ahead so the swing feet clear it.
    """
    def deco(spec, mcfg):
        z = np.tan(angle_rad) * x0 + radius * np.cos(angle_rad) + 0.003
        b = spec.worldbody.add_body()
        b.name = "boulder"
        b.pos = [x0, 0.0, z]
        # constrain the boulder to slide ALONG the slope (no flying off, no rolling
        # sideways) -- the robot shoves it up like a snow-plough, gravity pulls back.
        j = b.add_joint()
        j.name = "boulder_slide"
        j.type = mujoco.mjtJoint.mjJNT_SLIDE
        j.axis = [np.cos(angle_rad), 0.0, np.sin(angle_rad)]
        j.damping = 6.0
        g = b.add_geom()
        g.name = "boulder_geom"
        g.type = mujoco.mjtGeom.mjGEOM_SPHERE
        g.size = [radius, 0, 0]
        g.mass = mass
        g.rgba = [0.45, 0.45, 0.48, 1.0]
        g.friction = [0.9, 0.01, 0.001]
    return deco


# extended-arms-forward pose (to push a big ball at body height) per robot:
# joint-name -> angle. Overrides the tray crouch arms for this task.
_EXTEND_ARMS = {
    "g1": {f"{s}_shoulder_pitch_joint": -1.4 for s in ("left", "right")}
          | {f"{s}_shoulder_roll_joint": 0.0 for s in ("left", "right")}
          | {f"{s}_elbow_joint": 1.3 for s in ("left", "right")},
    "talos": {f"arm_{s}_2_joint": 0.1 for s in ("left", "right")}
             | {f"arm_{s}_4_joint": -0.2 for s in ("left", "right")},
}


def extend_arms(env, ctrl, robot):
    """Override the arm posture so the robot reaches both arms forward."""
    m, d = env.model, env.data
    act_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)]
    for j, ang in _EXTEND_ARMS.get(robot, {}).items():
        adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
        d.qpos[adr] = ang
        if j in act_names:
            ctrl.pos_task.q_nom[act_names.index(j)] = ang
    mujoco.mj_forward(m, d)


def run(angle_deg=5.0, mass=1.3, radius=0.36, robot="g1", viewer=False):
    params = load_params()
    params["gait"]["step_length"] = 0.14           # a determined push
    params["gait"]["n_steps"] = 8 if robot == "g1" else 5   # push a stretch, then stop
    alpha = np.deg2rad(angle_deg)
    x0 = radius + 0.42                              # ball touches the extended hands
    deco = boulder_decorator(alpha, x0, radius, mass)
    env, ctrl, terrain = build_on_terrain(params, "incline", angle_deg, None, robot, deco)
    extend_arms(env, ctrl, robot)                   # reach both arms forward to push
    # lean into the ball: a steady forward CoM bias counters the backward tipping
    # torque from pushing a big ball at hand height (like a person leaning to push).
    ctrl.slope_accel_ff = np.array([1.2 if robot == "g1" else 0.6, 0.0])
    settle(env, ctrl, terrain, 0.8)

    m, d = env.model, env.data
    bjid = m.body("boulder").id
    base = ctrl.base_id

    il = d.site_xpos[ctrl.left_site].copy()
    ir = d.site_xpos[ctrl.right_site].copy()
    com0 = d.subtree_com[base].copy()
    plan = WalkPlan(params, il, ir, com0, com_height=params["gait"]["com_height"],
                    gravity=params["env"]["gravity"], terrain=terrain)
    n = int(plan.duration / env.dt)
    bx0 = float(d.xpos[bjid][0])
    bz0 = float(d.xpos[bjid][2])
    log = {"t": [], "rx": [], "rz": [], "cx": []}
    fell = False

    def loop(i):
        ctrl.step(plan, i * env.dt)
        log["t"].append(i * env.dt)
        log["rx"].append(float(d.xpos[bjid][0]))
        log["rz"].append(float(d.xpos[bjid][2]))
        log["cx"].append(float(d.subtree_com[base][0]))
        return d.qpos[2] - terrain.height(d.subtree_com[base][0], 0.0) < 0.45

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

    pushed_x = float(d.xpos[bjid][0]) - bx0
    pushed_up = float(d.xpos[bjid][2]) - bz0
    print(f"\n===== Sisyphus ({robot.upper()}): push a {mass:.1f} kg boulder up {angle_deg:.0f} deg =====")
    print(f"fell             = {fell}")
    print(f"boulder pushed   = {pushed_x:.2f} m along x,  +{pushed_up*100:.0f} cm uphill")
    print(f"robot advanced   = {log['cx'][-1] - com0[0]:.2f} m")
    ok = (not fell) and pushed_x > 0.15
    print(f"\nRESULT: {'PASS — the rock went up!' if ok else 'FAIL'}")
    _plot(log, angle_deg, robot, mass, radius)
    return ok


def _plot(log, angle_deg, robot, mass, radius=0.3):
    from matplotlib.patches import Circle
    t = np.array(log["t"])
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    a = np.deg2rad(angle_deg)
    xs = np.linspace(0, max(log["rx"]) + 0.4, 100)
    ax[0].fill_between(xs, np.tan(a) * xs - 0.05, np.tan(a) * xs, color="0.75",
                       label=f"{angle_deg:.0f}° slope")
    ax[0].plot(log["cx"], np.tan(a) * np.array(log["cx"]), "b-", lw=1.5, label="robot (uphill)")
    ax[0].plot(log["rx"], log["rz"], "-", color="0.3", lw=1, alpha=0.5)
    # boulder as a circle at start (faint) and end (solid) -- shows its size
    ax[0].add_patch(Circle((log["rx"][0], log["rz"][0]), radius, color="0.6", alpha=0.4))
    ax[0].add_patch(Circle((log["rx"][-1], log["rz"][-1]), radius, color="0.4",
                           ec="0.2", lw=1.5, label="boulder (final)"))
    ax[0].set_aspect("equal"); ax[0].grid(True, alpha=0.4); ax[0].legend()
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("z [m]")
    ax[0].set_title(f"Sisyphus {robot.upper()}: boulder up the slope (side view)")
    ax[1].plot(t, np.array(log["rx"]) - log["rx"][0], "k", label="boulder Δx")
    ax[1].plot(t, np.array(log["cx"]) - log["cx"][0], "b--", label="robot Δx")
    ax[1].grid(True, alpha=0.4); ax[1].legend()
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("forward progress [m]")
    ax[1].set_title(f"{mass:.1f} kg boulder pushed uphill")
    out = Path(__file__).resolve().parents[1] / "logs" / f"sisyphus_{robot}_{angle_deg:.0f}deg.png"
    fig.tight_layout(); fig.savefig(out, dpi=115)
    print(f"plot saved: {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=5.0)
    ap.add_argument("--mass", type=float, default=1.3)
    ap.add_argument("--radius", type=float, default=0.36)
    ap.add_argument("--robot", default="g1", choices=["g1", "talos"])
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()
    run(args.angle, args.mass, args.radius, args.robot, args.viewer)
