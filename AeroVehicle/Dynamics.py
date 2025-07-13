import numpy as np
from Global.utils import wrap, rotation_matrix
from Global.simdata import UAVState, UAVForces, ActuatorOutputs


class VehicleForcesMoments:
    def __init__(self, vehicle_prop):
        self.vp = vehicle_prop
        self.output: UAVForces = UAVForces()

    def compute(self, current_state: UAVState, controls: ActuatorOutputs):
        u, v, w = current_state.x_vel, current_state.y_vel, current_state.z_vel
        phi, theta, psi = current_state.phi, current_state.theta, current_state.psi
        p, q, r = current_state.phi_rate, current_state.theta_rate, current_state.psi_rate

        # Aircraft properties
        m = self.vp["m"]
        S = self.vp["S"]
        b = self.vp["b"]
        c = self.vp["c"]
        rho = self.vp["rho"]
        l_q = self.vp.get("quad_arm_length", 0.5)  # distance from CG to each rotor
        k_yaw = self.vp.get("quad_yaw_coeff", 0.01)  # yaw torque constant

        alpha = wrap(np.arctan2(w, u), -np.pi, np.pi)
        V = np.linalg.norm([u, v, w])
        beta = wrap(np.arcsin(np.clip(v / V, -1, 1)), -np.pi / 2, np.pi / 2)
        q_dyn = 0.5 * rho * V**2 if V > 0 else 0.0

        # Control inputs
        aileron = controls.fw.aileron
        elevator = controls.fw.elevator
        rudder = controls.fw.rudder
        thrust_FW = controls.fw.throttle

        M1 = controls.quad.motor1  # LF (CCW)
        M2 = controls.quad.motor2  # RF (CW)
        M3 = controls.quad.motor3  # RB (CCW)
        M4 = controls.quad.motor4  # LB (CW)

        # Rotation matrices
        R_ned_to_body = rotation_matrix(phi, theta, psi)
        R_stb_to_body = rotation_matrix(0, -alpha, 0)

        # Gravity in body frame
        gravity_body = R_ned_to_body @ np.array([0, 0, m * 9.81])

        # Aerodynamic forces
        CL = self.vp["CL0"] + self.vp["CL_alpha"] * alpha + self.vp["CLq"] * q * c / (2 * V) + self.vp["CL_delta_e"] * elevator
        CD = self.vp["CD0"] + self.vp["CD_alpha"] * abs(alpha) + self.vp["CDq"] * abs(q) * c / (2 * V) + self.vp["CD_delta_e"] * abs(elevator)
        CY = self.vp["CY0"] + self.vp["CY_beta"] * beta + self.vp["CYp"] * p + self.vp["CYr"] * r + self.vp["CY_delta_a"] * aileron + self.vp["CY_delta_r"] * rudder

        lift = q_dyn * S * CL
        drag = q_dyn * S * CD
        F_aero_body = R_stb_to_body @ np.array([-drag, 0, -lift])
        F_aero_body[1] += q_dyn * S * CY

        # Thrust forces
        F_thrust_body = np.array([
            thrust_FW,
            0.0,
            -(M1 + M2 + M3 + M4)
        ])

        # Total forces
        Fx, Fy, Fz = F_aero_body + F_thrust_body + gravity_body

        # Aero moments
        Cl = self.vp["Cl0"] + self.vp["Cl_beta"] * beta + self.vp["Clp"] * p * b / (2 * V) + self.vp["Clr"] * r * b / (2 * V) + self.vp["Cl_delta_a"] * aileron + self.vp["Cl_delta_r"] * rudder
        Cm = self.vp["Cm0"] + self.vp["Cm_alpha"] * alpha + self.vp["Cmq"] * q * c / (2 * V) + self.vp["Cm_delta_e"] * elevator
        Cn = self.vp["Cn0"] + self.vp["Cn_beta"] * beta + (self.vp["Cnp"] * p + self.vp["Cnr"] * r) * b / (2 * V) + self.vp["Cn_delta_a"] * aileron + self.vp["Cn_delta_r"] * rudder

        l_aero = q_dyn * S * b * Cl
        m_aero = q_dyn * S * c * Cm
        n_aero = q_dyn * S * b * Cn

        # Moments from quad thrusts
        l_quad = l_q * ((M1 + M4) - (M2 + M3))  # Roll: left - right
        m_quad = l_q * ((M1 + M2) - (M3 + M4))  # Pitch: front - back
        n_quad = k_yaw * ((M1 + M3) - (M2 + M4))  # Yaw: CCW - CW

        # Total moments
        l_total = l_aero + l_quad
        m_total = m_aero + m_quad
        n_total = n_aero + n_quad

        self.output = UAVForces(lift, drag, Fx, Fy, Fz, l_total, m_total, n_total)
        return self.output
