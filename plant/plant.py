import numpy as np
from plant.kinematics import six_DOF_motion
from utils import wrap, rotation_matrix

class Simulation_class:
    def __init__(self, vehicle_prop):
        self.vehicle_prop = vehicle_prop  # Store vehicle properties
        return
    
    def simulate_one_step(self, input_state, motor_thrust, ctrl_srfc_deflection, dt):
        self.state = input_state
        #boundary conditions for propulsion and control surfaces
        min_thrust, max_thrust = 0, 200  # Adjust based on engine limits
        min_deflection, max_deflection = np.radians(-20), np.radians(20)  # Degrees for control surfaces

        # Wrap the filtered values within their respective bounds
        motor_thrust = [wrap(value, min_thrust, max_thrust) for value in motor_thrust]
        ctrl_srfc_deflection = [wrap(value, min_deflection, max_deflection) for value in ctrl_srfc_deflection]

        acc_body, omega_dot, forces_moments = six_DOF_motion(self.vehicle_prop, self.state, motor_thrust, ctrl_srfc_deflection)
        # Update velocity self.states
        self.state[3] += acc_body[0] * dt  # u (velocity in x-direction)
        self.state[4] += acc_body[1] * dt  # v (velocity in y-direction)
        self.state[5] += acc_body[2] * dt  # w (velocity in z-direction)

        # Update angular rate self.states
        self.state[9] += omega_dot[0] * dt  # p (angular velocity about x-axis)
        self.state[10] += omega_dot[1] * dt  # q (angular velocity about y-axis)
        self.state[11] += omega_dot[2] * dt  # r (angular velocity about z-axis)
        
        R_body_to_ned = rotation_matrix(self.state[6], self.state[7], self.state[8]).T 

        # Convert body frame velocities to NED frame
        V_ned = R_body_to_ned @ self.state[3:6]  # [u, v, w] in body frame to [Vx, Vy, Vz] in NED frame

        # Update position self.states using NED frame velocities
        self.state[0] += V_ned[0] * dt  # x (position in x-direction)
        self.state[1] += V_ned[1] * dt  # y (position in y-direction)
        self.state[2] += V_ned[2] * dt  # z (position in z-direction)

        self.state[6] = wrap(self.state[6] + self.state[9] * dt, -np.pi/2, np.pi/2)  # phi (roll angle)
        self.state[7] = wrap(self.state[7] + self.state[10] * dt, -np.pi/2, np.pi/2)  # theta (pitch angle)
        self.state[8] = wrap(self.state[8] + self.state[11] * dt,  -np.pi, np.pi)  # psi (yaw angle)

        return self.state, forces_moments

