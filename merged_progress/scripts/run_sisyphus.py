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


def run(angle_deg=6.0, mass=2.0, radius=0.12, robot="g1", viewer=False):
    params = load_params()
    params["gait"]["step_length"] = 0.14           # a determined push
    alpha = np.deg2rad(angle_deg)
    x0 = 0.30                                        # ball just ahead of the lead foot
    deco = boulder_decorator(alpha, x0, radius, mass)
    env, ctrl, terrain = build_on_terrain(params, "incline", angle_deg, None, robot, deco)
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
    _plot(log, angle_deg, robot, mass)
    return ok


def _plot(log, angle_deg, robot, mass):
    t = np.array(log["t"])
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    a = np.deg2rad(angle_deg)
    xs = np.linspace(0, max(log["rx"]) + 0.3, 100)
    ax[0].fill_between(xs, np.tan(a) * xs - 0.05, np.tan(a) * xs, color="0.75",
                       label=f"{angle_deg:.0f}° slope")
    ax[0].plot(log["cx"], np.tan(a) * np.array(log["cx"]), "b-", lw=1.5, label="robot (uphill)")
    ax[0].plot(log["rx"], log["rz"], "-", color="0.3", lw=2.2, label="boulder")
    ax[0].plot(log["rx"][-1], log["rz"][-1], "o", color="0.3", ms=10)
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
    ap.add_argument("--mass", type=float, default=2.0)
    ap.add_argument("--radius", type=float, default=0.12)
    ap.add_argument("--robot", default="g1", choices=["g1", "talos"])
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()
    run(args.angle, args.mass, args.radius, args.robot, args.viewer)
