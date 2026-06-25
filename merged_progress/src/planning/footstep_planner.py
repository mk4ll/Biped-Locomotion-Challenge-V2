"""Footstep planner: alternating steps + gait timeline.

Produces a list of footsteps (alternating left/right lanes, advancing in +x)
and the phase timeline consumed by the FSM / DCM / swing planners.
A kinematic clamp limits the per-step advance to a safe maximum.
"""
import numpy as np


class FootstepPlanner:
    def __init__(self, params):
        g = params["gait"]
        self.n_steps = g["n_steps"]
        self.step_length = g["step_length"]
        self.swing_apex = g["swing_apex"]
        self.t_ss = g["t_ss"]
        self.t_ds = g["t_ds"]
        self.t_ds_init = g["t_ds_init"]
        self.t_ds_final = g["t_ds_final"]
        self.first_swing = g["first_swing"]
        self.max_step = 1.5 * self.step_length     # kinematic clamp

    def plan(self, init_left, init_right, terrain=None):
        """init_left/right: xyz of the two foot sites. ``terrain`` (Terrain or None)
        gives terrain-aware foot placement: terrain.footstep_x snaps forward x
        (e.g. stair tread centres) and terrain.height sets the landing z so feet
        land ON the surface (flat / incline / stairs). None => flat ground.
        Returns (footsteps, timeline)."""
        L = np.array(init_left, float)
        R = np.array(init_right, float)
        # foot sites sit a fixed offset above the contact surface; preserve it.
        z_off = 0.5 * (L[2] + R[2]) - (terrain.height(0.5 * (L[0] + R[0]), 0.0)
                                       if terrain is not None else 0.0)

        def ground_z(x):
            h = terrain.height(x, 0.0) if terrain is not None else 0.0
            return h + z_off

        def next_x(stance_x, swing_x):
            if terrain is not None:
                return terrain.footstep_x(stance_x, swing_x, self.step_length)
            return stance_x + min(self.step_length, self.max_step)
        feet = {"left": L.copy(), "right": R.copy()}

        footsteps = [
            {"foot": "left", "pos": L.copy()},
            {"foot": "right", "pos": R.copy()},
        ]
        timeline = []
        t = 0.0

        def add(ph):
            ph["t0"] = t
            ph["dur"] = ph["dur"]
            ph["t1"] = t + ph["dur"]
            timeline.append(ph)
            return ph["t1"]

        # initial double support
        t = add({"type": "DS", "dur": self.t_ds_init, "support": "double",
                 "swing": None, "zmp_from": 0.5 * (L[:2] + R[:2]),
                 "zmp_to": 0.5 * (L[:2] + R[:2])})

        swing = self.first_swing
        for k in range(self.n_steps):
            support = "left" if swing == "right" else "right"
            target = feet[swing].copy()
            target[0] = next_x(feet[support][0], feet[swing][0])  # terrain-aware x
            target[2] = ground_z(target[0])                       # land ON the surface
            # SS: ZMP sits at the support foot.
            t = add({"type": "SS", "dur": self.t_ss, "support": support,
                     "swing": swing, "swing_from": feet[swing].copy(),
                     "swing_to": target.copy(),
                     "zmp_from": feet[support][:2].copy(),
                     "zmp_to": feet[support][:2].copy()})
            feet[swing] = target
            footsteps.append({"foot": swing, "pos": target.copy()})
            # DS: ZMP ramps from old support foot to the new stance (the foot just placed).
            next_support = swing       # the foot just placed becomes next support
            t = add({"type": "DS", "dur": self.t_ds, "support": "double",
                     "swing": None, "zmp_from": feet[support][:2].copy(),
                     "zmp_to": feet[next_support][:2].copy()})
            swing = "left" if swing == "right" else "right"

        # final settle: ZMP to mid-feet
        mid = 0.5 * (feet["left"][:2] + feet["right"][:2])
        t = add({"type": "DS", "dur": self.t_ds_final, "support": "double",
                 "swing": None, "zmp_from": timeline[-1]["zmp_to"].copy(),
                 "zmp_to": mid})
        return footsteps, timeline

    def plan_velocity(self, init_left, init_right, vx, vy, vyaw=0.0, terrain=None):
        """Omnidirectional footsteps from a body-frame velocity command.

        The support 'center' advances by (vx, vy) each step in the current heading
        frame and the heading rotates by vyaw per step; each foot is placed a fixed
        half-width to its side of the center. Handles forward / back / strafe /
        turn / curve. Returns (footsteps, timeline) with a per-phase 'heading'.
        """
        L = np.array(init_left, float)
        R = np.array(init_right, float)
        half_w = 0.5 * abs(L[1] - R[1])
        z_off = 0.5 * (L[2] + R[2]) - (terrain.height(0.5 * (L[0] + R[0]), 0.0)
                                       if terrain is not None else 0.0)

        def ground_z(xy):
            h = terrain.height(xy[0], xy[1]) if terrain is not None else 0.0
            return h + z_off

        def rot(th):
            c, s = np.cos(th), np.sin(th)
            return np.array([[c, -s], [s, c]])

        theta = 0.0
        center = 0.5 * (L[:2] + R[:2])
        feet = {"left": L.copy(), "right": R.copy()}
        footsteps = [{"foot": "left", "pos": L.copy()},
                     {"foot": "right", "pos": R.copy()}]
        timeline = []
        t = 0.0
        T_step = self.t_ss + self.t_ds

        def add(ph):
            ph["t0"] = t; ph["t1"] = t + ph["dur"]
            ph.setdefault("heading", theta)
            timeline.append(ph)
            return ph["t1"]

        t = add({"type": "DS", "dur": self.t_ds_init, "support": "double",
                 "swing": None, "zmp_from": 0.5 * (L[:2] + R[:2]),
                 "zmp_to": 0.5 * (L[:2] + R[:2]), "heading": theta})

        swing = self.first_swing
        for k in range(self.n_steps):
            support = "left" if swing == "right" else "right"
            center = center + rot(theta) @ np.array([vx, vy]) * T_step
            theta = theta + vyaw * T_step
            lateral = half_w if swing == "left" else -half_w
            txy = center + rot(theta) @ np.array([0.0, lateral])
            target = np.array([txy[0], txy[1], ground_z(txy)])
            t = add({"type": "SS", "dur": self.t_ss, "support": support,
                     "swing": swing, "swing_from": feet[swing].copy(),
                     "swing_to": target.copy(),
                     "zmp_from": feet[support][:2].copy(),
                     "zmp_to": feet[support][:2].copy(), "heading": theta})
            feet[swing] = target
            footsteps.append({"foot": swing, "pos": target.copy()})
            t = add({"type": "DS", "dur": self.t_ds, "support": "double",
                     "swing": None, "zmp_from": feet[support][:2].copy(),
                     "zmp_to": feet[swing][:2].copy(), "heading": theta})
            swing = "left" if swing == "right" else "right"

        mid = 0.5 * (feet["left"][:2] + feet["right"][:2])
        t = add({"type": "DS", "dur": self.t_ds_final, "support": "double",
                 "swing": None, "zmp_from": timeline[-1]["zmp_to"].copy(),
                 "zmp_to": mid, "heading": theta})
        return footsteps, timeline
