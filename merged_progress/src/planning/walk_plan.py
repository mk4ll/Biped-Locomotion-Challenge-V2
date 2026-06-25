"""Orchestrates footsteps + FSM + DCM CoM + swing into a single reference(t).

This is the planner output consumed by the WBC in Stage 4. It is completely
independent of the controller (DESCRIPTION.md s2): it only needs the initial
foot positions and CoM height.
"""
import numpy as np

from src.planning.footstep_planner import FootstepPlanner
from src.planning.com_planner import DCMPlanner
from src.planning.swing_planner import swing_trajectory
from src.planning.fsm import phase_at, total_duration


class WalkPlan:
    def __init__(self, params, init_left, init_right, com0, com_height=None,
                 gravity=9.81, terrain=None, velocity=None):
        self.params = params
        self.terrain = terrain
        # nominal CoM height ABOVE the local surface (regulated relative to terrain)
        h0 = terrain.height(com0[0], 0.0) if terrain is not None else 0.0
        self.z_above = (com_height if com_height is not None else com0[2]) - h0
        self.fp = FootstepPlanner(params)
        if velocity is not None:
            vx, vy, vyaw = velocity
            self.footsteps, self.timeline = self.fp.plan_velocity(
                init_left, init_right, vx, vy, vyaw, terrain=terrain)
        else:
            self.footsteps, self.timeline = self.fp.plan(init_left, init_right,
                                                         terrain=terrain)
        # DCM uses the vertical CoM height above the surface; mild slope ok.
        self.dcm = DCMPlanner(params, self.z_above, gravity)
        self.traj = self.dcm.generate(self.timeline, np.asarray(com0)[:2])
        self.dt = self.traj["t"][1] - self.traj["t"][0]
        self.duration = total_duration(self.timeline)

    def _ground(self, x):
        return self.terrain.height(x, 0.0) if self.terrain is not None else 0.0

    # -- sampled trajectory access -------------------------------------------
    def _idx(self, t):
        return int(np.clip(round(t / self.dt), 0, len(self.traj["t"]) - 1))

    def reference(self, t):
        """Full reference at time t for the WBC."""
        i = self._idx(t)
        com_x = self.traj["com"][i, 0]
        com_z = self._ground(com_x) + self.z_above             # follow the terrain
        com = np.array([com_x, self.traj["com"][i, 1], com_z])
        com_vx = self.traj["com_vel"][i, 0]
        # vertical CoM rate from terrain slope dh/dx (finite diff, robust on stairs)
        dh = (self._ground(com_x + 1e-3) - self._ground(com_x - 1e-3)) / 2e-3
        com_vel = np.array([com_vx, self.traj["com_vel"][i, 1], com_vx * dh])
        zmp = self.traj["zmp"][i].copy()
        dcm = self.traj["dcm"][i].copy()
        ph, s = phase_at(self.timeline, t)
        ref = {"com": com, "com_vel": com_vel, "zmp": zmp, "dcm": dcm,
               "omega": self.traj["omega"], "progress": s,
               "support": ph["support"], "phase": ph["type"],
               "swing": ph["swing"], "swing_pos": None, "swing_vel": None,
               "heading": ph.get("heading", 0.0)}
        if ph["type"] == "SS":
            pos, vel = swing_trajectory(s, ph["swing_from"], ph["swing_to"],
                                        self.fp.swing_apex, ph["dur"])
            ref["swing_pos"] = pos
            ref["swing_vel"] = vel
        return ref
