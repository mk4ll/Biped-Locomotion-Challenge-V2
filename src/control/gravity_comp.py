"""Stage 1 -- gravity compensation (static Inverse Dynamics).

At static equilibrium (v = 0, v̇ = 0) the floating-base dynamics reduce to
    Sᵀ τ + Σ_c J_cᵀ f_c = h ,      h = qfrc_bias = g(q) .
The 6 unactuated base rows must be met by contact forces alone:
    (J_cᵀ f)[base] = h[base] .
We choose contact forces closest to an even vertical weight distribution
(equality-constrained least squares), then read off the joint torques from
the actuated rows:
    τ = (h − J_cᵀ f)[actuated dofs] .

Recomputed every control step from the current state, so it is quasi-static
feedback and naturally rejects small numerical drift.
"""
import numpy as np


class GravityCompensator:
    def __init__(self, terms, total_mass, gravity=9.81, reg=1e-6):
        self.terms = terms
        self.weight = total_mass * gravity
        self.reg = reg

    def compute(self, data, active_site_ids=None):
        """Return (tau [nu], f [3*ncp]) holding the robot in static balance."""
        T = self.terms
        h = T.bias(data)                                   # (nv,)
        Jc = T.contact_jacobian(data, active_site_ids)     # (3*ncp, nv)
        ncp = Jc.shape[0] // 3

        base = np.arange(6)                                # unactuated base dofs
        A = Jc[:, base].T                                  # (6, 3*ncp)
        b = h[base]                                        # (6,)

        # Nominal: support total weight evenly, vertical (world +z).
        f0 = np.zeros(3 * ncp)
        f0[2::3] = self.weight / max(ncp, 1)

        # Equality-constrained least squares: min ||f - f0|| s.t. A f = b.
        # f = f0 + Aᵀ (A Aᵀ + reg I)^-1 (b - A f0).
        AAT = A @ A.T + self.reg * np.eye(A.shape[0])
        lam = np.linalg.solve(AAT, b - A @ f0)
        f = f0 + A.T @ lam

        # Actuated rows give the joint torques.
        gen_contact = Jc.T @ f                             # (nv,)
        tau = (h - gen_contact)[T.act_dof]                 # (nu,)
        return tau, f

    def residual(self, data, tau, f, active_site_ids=None):
        """Dynamics residual ‖Sᵀτ + Jᵀf − h‖ (should be ~0)."""
        T = self.terms
        h = T.bias(data)
        Jc = T.contact_jacobian(data, active_site_ids)
        lhs = T.S.T @ tau + Jc.T @ f
        return np.linalg.norm(lhs - h)
