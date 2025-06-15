import numpy as np
from Global.simdata import UAVForce_class, UAVState_class
from dataclasses import astuple

class SixDOFDynamics:
    def __init__(self, vehicle_prop):
        self.vp = vehicle_prop

    def compute(self, current_state: UAVState_class, forces_moments: UAVForce_class):
        u, v, w = current_state.x_vel, current_state.y_vel, current_state.z_vel
        p, q, r = current_state.phi_rate, current_state.theta_rate, current_state.psi_rate

        (_, _, Fx, Fy, Fz, l, m, n) = astuple(forces_moments)

        m_uav = self.vp["m"]
        Jx, Jy, Jz, Jxz = self.vp["Jx"], self.vp["Jy"], self.vp["Jz"], self.vp["Jxz"]

        # Translational accelerations
        u_dot = r*v - q*w + Fx / m_uav
        v_dot = p*w - r*u + Fy / m_uav
        w_dot = q*u - p*v + Fz / m_uav

        # Angular accelerations
        gamma = Jx * Jz - Jxz**2
        gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
        gamma2 = (Jz * (Jz - Jy) + Jxz**2) / gamma
        gamma3 = Jz / gamma
        gamma4 = Jxz / gamma
        gamma5 = (Jz - Jx) / Jy
        gamma6 = Jxz / Jy
        gamma7 = ((Jx - Jy) * Jx + Jxz**2) / gamma
        gamma8 = Jx / gamma

        p_dot = gamma1 * p * q - gamma2 * q * r + gamma3 * l + gamma4 * n
        q_dot = gamma5 * p * r - gamma6 * (p**2 - r**2) + m / Jy
        r_dot = gamma7 * p * q - gamma1 * q * r + gamma4 * l + gamma8 * n

        return np.array([u_dot, v_dot, w_dot]), np.array([p_dot, q_dot, r_dot])
