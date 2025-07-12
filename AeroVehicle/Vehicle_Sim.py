import numpy as np
from Global.utils import linear_scale, wrap, rotation_matrix
from AeroVehicle.Kinematics import SixDOFDynamics
from AeroVehicle.Dynamics import VehicleForcesMoments
from Global.simdata import UAVState, UAVForces, ActuatorOutputs
from AeroVehicle.actuators import Actuator_model


class UAVSimulation:
    def __init__(self, vehicle_prop, dt):
        self.vehicle_prop = vehicle_prop
        self.dt = dt
        self.min_thrust, self.max_thrust = 0, 110 * 0.3
        D2R = np.pi / 180
        self.min_deflection, self.max_deflection = -30 * D2R, 30 * D2R

        self.dynamics = VehicleForcesMoments(vehicle_prop)
        self.kinematics = SixDOFDynamics(vehicle_prop)
        self.actuators = Actuator_model(self.min_thrust, self.max_thrust, self.min_deflection, self.max_deflection)

        self.controls : ActuatorOutputs = ActuatorOutputs()
        self.forces_moments : UAVForces = UAVForces()
        self.output : UAVState = UAVState()


    def simulate_one_step(self, current_state: UAVState, control_input: ActuatorOutputs ):

        # Unpack state for clarity
        u, v, w = current_state.x_vel, current_state.y_vel, current_state.z_vel
        phi, theta, psi = current_state.phi, current_state.theta, current_state.psi
        p, q, r = current_state.phi_rate, current_state.theta_rate, current_state.psi_rate

        # Compute forces and dynamics
        self.forces_moments = self.dynamics.compute(current_state, self.controls)
        acc_body, omega_dot = self.kinematics.compute(current_state, self.forces_moments)

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
        self.output.x = current_state.x + V_ned[0] * self.dt
        self.output.y = current_state.y + V_ned[1] * self.dt
        self.output.z = current_state.z + V_ned[2] * self.dt

        # Update full state
        self.output.x_vel = u
        self.output.y_vel = v
        self.output.z_vel = w

        self.output.phi = phi
        self.output.theta = theta
        self.output.psi = psi

        self.output.phi_rate = p
        self.output.theta_rate = q
        self.output.psi_rate = r

        return self.output,  self.forces_moments
