import numpy as np
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle
from AeroVehicle.Kinematics import six_DOF_motion
from Global.Utils import wrap, rotation_matrix, linear_scale


class UAVSimulation:
    def __init__(self, vehicle_prop, dt):
        self.vehicle_prop = vehicle_prop
        self.dt = dt
        self.state = np.zeros(12)  # [x, y, z, u, v, w, phi, theta, psi, p, q, r]
        self.min_thrust, self.max_thrust = 0, 110  # Thrust limits
        self.min_deflection, self.max_deflection = np.deg2rad(-30), np.deg2rad(30)  # Deflection limits

    def simulate_one_step(self, input_state, control_input):
        # Unpack control inputs
        motor_thrust, ctrl_srfc_deflection = control_input[0:5], control_input[5:]

        # Unpack state
        self.state[:] = input_state  # shallow copy for safety
        # Unpack state components
        u, v, w = self.state[3:6]
        phi, theta, psi = self.state[6:9]
        p, q, r = self.state[9:12]

        for i in range(len(motor_thrust)):
            motor_thrust[i] = linear_scale(motor_thrust[i], in_min=1100, in_max=2000, out_min=self.min_thrust, out_max=self.max_thrust)

        for i in range(len(ctrl_srfc_deflection)):
            ctrl_srfc_deflection = linear_scale(ctrl_srfc_deflection, in_min=1100, in_max=2000, out_min=self.min_deflection, out_max=self.max_deflection)

        # Dynamics
        acc_body, omega_dot, forces_moments = six_DOF_motion(self.vehicle_prop, self.state, motor_thrust, ctrl_srfc_deflection)

        # Integrate linear velocities
        u += acc_body[0] * self.dt
        v += acc_body[1] * self.dt
        w += acc_body[2] * self.dt

        # Integrate angular rates
        p += omega_dot[0] * self.dt
        q += omega_dot[1] * self.dt
        r += omega_dot[2] * self.dt

        # Update orientation
        phi = wrap(phi + p * self.dt, -np.pi, np.pi)
        theta = wrap(theta + q * self.dt, -np.pi, np.pi)
        psi = wrap(psi + r * self.dt, -np.pi, np.pi)

        # Velocity in NED frame
        R_body_to_ned = rotation_matrix(phi, theta, psi).T
        V_body = np.array([u, v, w])
        V_ned = R_body_to_ned @ V_body

        # Integrate position
        self.state[0:3] += V_ned * self.dt

        # Write updated values back to state
        self.state[3:6] = [u, v, w]
        self.state[6:9] = [phi, theta, psi]
        self.state[9:12] = [p, q, r]

        return self.state, forces_moments
