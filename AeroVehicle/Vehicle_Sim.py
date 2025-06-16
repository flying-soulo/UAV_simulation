import numpy as np
from Global.utils import linear_scale, wrap, rotation_matrix
from AeroVehicle.Kinematics import SixDOFDynamics
from AeroVehicle.Dynamics import VehicleForcesMoments
from Global.simdata import UAVState_class, UAVForce_class, Actuator_class

class UAVSimulation:
    def __init__(self, vehicle_prop, dt):
        self.vehicle_prop = vehicle_prop
        self.dt = dt
        self.min_thrust, self.max_thrust = 0, 110 * 0.3
        D2R = np.pi / 180
        self.min_deflection, self.max_deflection = -30 * D2R, 30 * D2R

        self.dynamics = VehicleForcesMoments(vehicle_prop)
        self.kinematics = SixDOFDynamics(vehicle_prop)
        self.controls : Actuator_class = Actuator_class()
        self.forces_moments : UAVForce_class = UAVForce_class()
        self.updated_state : UAVState_class = UAVState_class()


    def simulate_one_step(self, current_state: UAVState_class, control_input: Actuator_class ):

        # Scale motor thrusts and control surface deflections
        self.controls.Quad.Motor1 = linear_scale(control_input.Quad.Motor1, 1100, 2000, self.min_thrust, self.max_thrust)
        self.controls.Quad.Motor2 = linear_scale(control_input.Quad.Motor2, 1100, 2000, self.min_thrust, self.max_thrust)
        self.controls.Quad.Motor3 = linear_scale(control_input.Quad.Motor3, 1100, 2000, self.min_thrust, self.max_thrust)
        self.controls.Quad.Motor4 = linear_scale(control_input.Quad.Motor4, 1100, 2000, self.min_thrust, self.max_thrust)
        self.controls.FW.throttle = linear_scale(control_input.FW.throttle, 1100, 2000, self.min_thrust, self.max_thrust)
        self.controls.FW.aileron = linear_scale(self.controls.FW.aileron, 1100, 2000, self.min_deflection, self.max_deflection)
        self.controls.FW.elevator = linear_scale(self.controls.FW.elevator, 1100, 2000, self.min_deflection, self.max_deflection)
        self.controls.FW.rudder= linear_scale(self.controls.FW.rudder, 1100, 2000, self.min_deflection, self.max_deflection)

        # Compute forces and dynamics
        self.forces_moments = self.dynamics.compute(current_state, self.controls)
        acc_body, omega_dot = self.kinematics.compute(current_state, self.forces_moments)

        # Unpack state for clarity
        u, v, w = current_state.x_vel, current_state.y_vel, current_state.z_vel
        phi, theta, psi = current_state.phi, current_state.theta, current_state.psi
        p, q, r = current_state.phi_rate, current_state.theta_rate, current_state.psi_rate

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
        self.updated_state.x = current_state.x + V_ned[0] * self.dt
        self.updated_state.y = current_state.y + V_ned[1] * self.dt
        self.updated_state.z = current_state.z + V_ned[2] * self.dt

        # Update full state
        self.updated_state.x_vel = u
        self.updated_state.y_vel = v
        self.updated_state.z_vel = w

        self.updated_state.phi = phi
        self.updated_state.theta = theta
        self.updated_state.psi = psi

        self.updated_state.phi_rate = p
        self.updated_state.theta_rate = q
        self.updated_state.psi_rate = r

        return self.updated_state,  self.forces_moments
