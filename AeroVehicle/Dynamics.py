import numpy as np
from Global.Utils import wrap, rotation_matrix
from Global.simdata import UAVState_class, UAVForce_class, Actuator_class


class VehicleForcesMoments:
    def __init__(self, vehicle_prop):
        self.vp = vehicle_prop
        self.output : UAVForce_class

    def compute(self, current_state: UAVState_class, controls: Actuator_class):
        """
        Compute the forces and moments acting on the vehicle.
        Args:
        state: Current state of the vehicle [x, y, z, u, v, w, phi, theta, psi, p, q, r]
        motor_thrust: Thrust from the motors [thrust_LF, thrust_RF, thrust_RB, thrust_LB, thrust_FW]

        """

        # Unpack state variables
        u, v, w = current_state.x_vel, current_state.y_vel, current_state.z_vel
        phi, theta, psi = current_state.phi, current_state.theta, current_state.psi
        p, q, r = current_state.phi_rate, current_state.theta_rate, current_state.psi_rate

        # Unpack vehicle properties
        m = self.vp["m"]
        S = self.vp["S"]
        b = self.vp["b"]
        c = self.vp["c"]
        rho = self.vp["rho"]

        alpha = wrap(np.arctan2(w, u), -np.pi, np.pi)
        V = np.linalg.norm([u, v, w])
        beta = wrap(np.arcsin(np.clip(v / V, -1, 1)), -np.pi / 2, np.pi / 2)
        q_dyn = 0.5 * rho * V**2

        aileron, elevator, rudder = controls.FW_aileron, controls.FW_elevator, controls.FW_rudder
        thrust_FW =  controls.FW_throttle
        thrust_LF, thrust_RF, thrust_RB, thrust_LB = controls.Quad_Motor1, controls.Quad_Motor2, controls.Quad_Motor3, controls.Quad_Motor4

        R_ned_to_body = rotation_matrix(phi, theta, psi)
        R_stb_to_body = rotation_matrix(0, -alpha, 0)
        gravity_body = R_ned_to_body @ np.array([0, 0, m * 9.81])

        # Aerodynamic forces
        CL = (
            self.vp["CL0"]
            + self.vp["CL_alpha"] * alpha
            + self.vp["CLq"] * q * c / (2 * V)
            + self.vp["CL_delta_e"] * elevator
        )
        CD = (
            self.vp["CD0"]
            + self.vp["CD_alpha"] * abs(alpha)
            + self.vp["CD_delta_e"] * abs(elevator)
            + self.vp["CDq"] * abs(q) * c / (2 * V)
        )
        CY = (
            self.vp["CY0"]
            + self.vp["CY_beta"] * beta
            + self.vp["CYp"] * p
            + self.vp["CYr"] * r
            + self.vp["CY_delta_a"] * aileron
            + self.vp["CY_delta_r"] * rudder
        )

        lift = q_dyn * S * CL
        drag = q_dyn * S * CD
        F_aero_body = R_stb_to_body @ np.array([-drag, 0, -lift])
        F_aero_body[1] += q_dyn * S * CY

        # Thrust
        thrust_body = np.array([thrust_FW, 0, -(thrust_LB + thrust_LF + thrust_RB + thrust_LB)])
        Fx, Fy, Fz = F_aero_body + gravity_body + thrust_body

        # Moments
        Cl = (
            self.vp["Cl0"]
            + self.vp["Cl_beta"] * beta
            + self.vp["Clp"] * p * b / (2 * V)
            + self.vp["Clr"] * r * b / (2 * V)
            + self.vp["Cl_delta_a"] * aileron
            + self.vp["Cl_delta_r"] * rudder
        )
        Cm = (
            self.vp["Cm0"]
            + self.vp["Cm_alpha"] * alpha
            + self.vp["Cmq"] * q * c / (2 * V)
            + self.vp["Cm_delta_e"] * elevator
        )
        Cn = (
            self.vp["Cn0"]
            + self.vp["Cn_beta"] * beta
            + (self.vp["Cnp"] * p + self.vp["Cnr"] * r) * b / (2 * V)
            + self.vp["Cn_delta_a"] * aileron
            + self.vp["Cn_delta_r"] * rudder
        )

        l = q_dyn * S * b * Cl
        m = q_dyn * S * c * Cm
        n = q_dyn * S * b * Cn

        self.output = UAVForce_class(lift, drag, Fx, Fy, Fz, l, m, n)
        return self.output
