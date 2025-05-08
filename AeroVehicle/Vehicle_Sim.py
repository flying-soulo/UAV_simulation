import numpy as np
from Global.Utils import linear_scale, wrap, rotation_matrix
from AeroVehicle.Kinematics import SixDOFDynamics
from AeroVehicle.Dynamics import VehicleForcesMoments

class UAVSimulation:
    def __init__(self, vehicle_prop, dt):
        self.vehicle_prop = vehicle_prop
        self.dt = dt
        self.state = np.zeros(12)  # [x, y, z, u, v, w, phi, theta, psi, p, q, r]
        self.min_thrust, self.max_thrust = 0, 110 * 0.3  # Adjusted thrust range
        D2R = np.pi/180
        self.min_deflection, self.max_deflection = -30*D2R, 30*D2R

        self.dynamics = VehicleForcesMoments(vehicle_prop)
        self.kinematics = SixDOFDynamics(vehicle_prop)

    def simulate_one_step(self, input_state, control_input):
        self.state = input_state.copy()

        motor_thrust = control_input[0:5]
        ctrl_srfc_deflection = control_input[5:]

        # Scale motor thrusts and control surface deflections
        motor_thrust = [
            linear_scale(t, 1100, 2000, self.min_thrust, self.max_thrust) for t in motor_thrust
        ]
        ctrl_srfc_deflection = [
            linear_scale(d, 1100, 2000, self.min_deflection, self.max_deflection) for d in ctrl_srfc_deflection
        ]

        # Compute forces and dynamics
        forces_moments = self.dynamics.compute(self.state, motor_thrust, ctrl_srfc_deflection)
        acc_body, omega_dot = self.kinematics.compute(self.state, forces_moments)

        # Unpack state for clarity
        u, v, w = self.state[3:6]
        phi, theta, psi = self.state[6:9]
        p, q, r = self.state[9:12]

        # Integrate velocities
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

        # Compute inertial velocity and update position
        V_body = np.array([u, v, w])
        R_ned_to_body = rotation_matrix(phi, theta, psi)
        V_ned = R_ned_to_body.T @ V_body
        self.state[0:3] += V_ned * self.dt

        # Update full state
        self.state[3:6] = [u, v, w]
        self.state[6:9] = [phi, theta, psi]
        self.state[9:12] = [p, q, r]

        return self.state.copy(), forces_moments
