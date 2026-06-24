"""Floating-base dynamics terms extracted from MuJoCo.

Provides, per control step:
  M(q)          mass matrix              (nv x nv)   via mj_fullM
  h(q,v)        bias = C(q,v)v + g(q)    (nv,)       via data.qfrc_bias
  S             selection matrix         (nu x nv)
  J_c           stacked contact Jacobian (3*ncp x nv) at foot corner points
  Jdot_v        (J̇_c v)                 (3*ncp,)    drift term

All quantities follow:  M v̇ + h = Sᵀ τ + Σ_c J_cᵀ f_c .
Contact points are the foot corner SITES (world-frame 3D point contacts).
"""
import mujoco
import numpy as np


class ModelTerms:
    def __init__(self, model, contact_sites):
        """contact_sites: list of site names used as point contacts."""
        self.model = model
        self.nv = model.nv
        self.nu = model.nu
        self.site_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s)
                         for s in contact_sites]
        self.site_names = list(contact_sites)
        # Selection matrix (actuator -> dof).
        self.S = np.zeros((self.nu, self.nv))
        self.act_dof = np.empty(self.nu, dtype=int)
        for a in range(self.nu):
            dof = model.jnt_dofadr[model.actuator_trnid[a, 0]]
            self.S[a, dof] = 1.0
            self.act_dof[a] = dof
        # Reusable buffers for mj_jac.
        self._jacp = np.zeros((3, self.nv))
        self._jacr = np.zeros((3, self.nv))

    # -- inertia / bias -------------------------------------------------------
    def mass_matrix(self, data) -> np.ndarray:
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(self.model, M, data.qM)
        return M

    def bias(self, data) -> np.ndarray:
        # qfrc_bias = C(q,v) v + g(q)  (Coriolis/centrifugal + gravity).
        return data.qfrc_bias.copy()

    # -- contacts -------------------------------------------------------------
    def site_jacobian(self, data, site_id) -> np.ndarray:
        """Translational Jacobian (3 x nv) of a site in world frame."""
        mujoco.mj_jacSite(self.model, data, self._jacp, self._jacr, site_id)
        return self._jacp.copy()

    def contact_jacobian(self, data, active_site_ids=None) -> np.ndarray:
        """Stack 3xnv translational Jacobians of the given contact sites.

        active_site_ids: subset of self.site_ids in contact (default: all).
        Returns (3*n x nv).
        """
        ids = self.site_ids if active_site_ids is None else active_site_ids
        if len(ids) == 0:
            return np.zeros((0, self.nv))
        return np.vstack([self.site_jacobian(data, sid) for sid in ids])

    def contact_jdot_v(self, data, active_site_ids=None) -> np.ndarray:
        """Drift term J̇_c v for each contact point (3*n,).

        Spatial site acceleration = J q̈ + J̇ v.  Setting q̈ = 0 isolates J̇ v.
        We temporarily zero qacc and recompute the body accelerations
        (mj_rnePostConstraint), read the site accel, then restore. With v = 0
        (static) this is exactly 0, as expected.
        """
        ids = self.site_ids if active_site_ids is None else active_site_ids
        if len(ids) == 0:
            return np.zeros(0)
        saved = data.qacc.copy()
        data.qacc[:] = 0.0
        mujoco.mj_rnePostConstraint(self.model, data)
        out = []
        acc = np.zeros(6)
        for sid in ids:
            mujoco.mj_objectAcceleration(self.model, data, mujoco.mjtObj.mjOBJ_SITE,
                                         sid, acc, 0)  # 0 -> world frame; [ang; lin]
            out.append(acc[3:6].copy())  # linear part = J̇v for this point
        data.qacc[:] = saved
        mujoco.mj_rnePostConstraint(self.model, data)  # restore cacc
        return np.concatenate(out)

    def site_pos(self, data, site_id) -> np.ndarray:
        return data.site_xpos[site_id].copy()
