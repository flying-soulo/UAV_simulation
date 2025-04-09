import numpy as np
from Autopilot.PID import PID_class

class MasterController:
    """
    Aggregates individual controllers for quadrotor and fixed-wing modes.
    """
    def __init__(self):
        # Quadrotor controller
        self.quad_x_pos = PID_class()
        self.quad_x_vel = PID_class()
        self.quad_y_pos = PID_class()
        self.quad_y_vel = PID_class()
        self.quad_phi = PID_class()
        self.quad_phi_rate = PID_class()
        self.quad_theta = PID_class()
        self.quad_theta_rate = PID_class()
        self.quad_psi = PID_class()
        self.quad_psi_rate = PID_class()
        self.quad_z_pos = PID_class()
        self.quad_z_vel = PID_class()
        self.quad_z_accel = PID_class()

        # Fixed-wing controller
        self.fw_airspeed  = PID_class()
        self.fw_heading  = PID_class()
        self.fw_altitude  = PID_class()
        self.fw_roll  = PID_class()
        self.fw_pitch  = PID_class()
        self.fw_yaw  = PID_class()
        self.fw_roll_rate  = PID_class()
        self.fw_pitch_rate = PID_class()
        self.fw_yaw_rate  = PID_class()

        #outputs
        self.throttle = np.zeros(5)
        self.ctrl_srfc_pwm = np.zeros(3)
    
    # Quadrotor control methods
    def run_x_position(self, target_x, current_x, current_x_vel):
        target_x_vel = self.quad_x_pos.run_pid(target_x, current_x)
        target_x_accel = self.quad_x_vel.run_pid(target_x_vel, current_x_vel)
        return target_x_accel

    def run_y_position(self, target_y, current_y, current_y_vel):
        target_y_vel = self.quad_y_pos.run_pid(target_y, current_y)
        target_y_accel = self.quad_y_vel.run_pid(target_y_vel, current_y_vel)
        return target_y_accel

    def run_altitude(self, target_z, current_z, current_z_vel, current_z_accel):
        target_z_vel = self.quad_z_pos.run_pid(target_z, current_z)
        target_z_accel = self.quad_z_vel.run_pid(target_z_vel, current_z_vel)
        target_z_jerk = self.quad_z_accel.run_pid(target_z_accel, current_z_accel)
        return target_z_jerk

    def run_phi(self, target_phi, current_phi, current_phi_rate):
        target_phi_rate = self.quad_phi.run_pid(target_phi, current_phi)
        target_phi_accel = self.quad_phi_rate.run_pid(target_phi_rate, current_phi_rate)
        return target_phi_accel

    def run_theta(self, target_theta, current_theta, current_theta_rate):
        target_theta_rate = self.quad_theta.run_pid(target_theta, current_theta)
        target_theta_accel = self.quad_theta_rate.run_pid(target_theta_rate, current_theta_rate)
        return target_theta_accel

    def run_psi(self, target_psi, current_psi, current_psi_rate):
        target_psi_rate = self.quad_psi.run_pid(target_psi, current_psi)
        target_psi_accel = self.quad_psi_rate.run_pid(target_psi_rate, current_psi_rate)
        return target_psi_accel
    
    def run_quad_mode(self, current_state, target_x, target_y, target_z, target_heading, dt):
        # Extract relevant state variables
        current_x = current_state[0]  # x position
        current_y = current_state[1]  # y position
        current_z = current_state[2]  # z position (altitude)
        current_x_vel = current_state[3]  # x velocity
        current_y_vel = current_state[4]  # y velocity
        current_z_vel = current_state[5]  # z velocity
        current_attitude = current_state[6:9]  # attitude (phi, theta, psi)
        current_rates = current_state[9:12]  # body rates (p, q, r)
        current_z_accel = current_state[15]

        # Compute target roll (phi) from x position controller
        target_phi = self.run_x_position(target_x, current_x, current_x_vel, dt)

        # Compute target pitch (theta) from y position controller
        target_theta = self.run_y_position(target_y, current_y, current_y_vel, dt)

        # Compute roll throttle from phi controller
        roll_throttle = self.run_phi(target_phi, current_attitude[0], current_rates[0], dt)

        # Compute pitch throttle from theta controller
        pitch_throttle = self.run_theta(target_theta, current_attitude[1], current_rates[1], dt)

        # Compute yaw throttle from psi controller
        yaw_throttle = self.run_psi(target_heading, current_attitude[2], current_rates[2], dt)

        # Compute base throttle from z position controller
        throttle_base = self.run_altitude(target_z, current_z, current_z_vel, current_z_accel, dt)
        
        ###############################################################
            # Combine throttle outputs for quadrotor motors
            # Assuming the following motor configuration (top view):
            #
            #        Front
            #          ^
            #          |
            #     1 CCW| CW 0
            #          |
            #   -------+-------
            #          |
            #     2 CW | CCW 3
            #          |
            #        Back
            #
            # Motor 0: Right Front (CW)
            # Motor 1: Left Front (CCW)
            # Motor 2: Left Back (CCW)
            # Motor 3: Right Back (CW)

        self.throttle[0] = (throttle_base - roll_throttle - pitch_throttle - yaw_throttle)  # Left Front (CCW)
        self.throttle[1] = (throttle_base + roll_throttle - pitch_throttle + yaw_throttle)  # Right Front (CW)
        self.throttle[2] = (throttle_base + roll_throttle + pitch_throttle - yaw_throttle)  # Right Back (CW)
        self.throttle[3] = (throttle_base - roll_throttle + pitch_throttle + yaw_throttle)  # Left Back (CCW)

        self.throttle[4] = 0  # No fixed-wing throttle in quad mode
        self.ctrl_srfc_pwm = np.zeros(3)  # No control surfaces for quadrotor

        return self.throttle, self.ctrl_srfc_pwm

    def throttle_controller(self, target_airspeed, current_airspeed, dt):
        throttle_command = self.fw_airspeed.run_pid(target_airspeed, current_airspeed, dt)
        return throttle_command

    def elevator_controller(self, target_altitude, current_altitude, current_pitch, current_pitch_rate, dt):
        pitch_command = self.fw_altitude.run_pid(target_altitude, current_altitude, dt)
        pitch_command = -pitch_command # due to alttiude up negative
        pitch_rate_command = self.fw_pitch.run_pid(pitch_command, current_pitch, dt)
        
        elevator_deflection = self.fw_pitch_rate.run_pid(pitch_rate_command, current_pitch_rate, dt)
        elevator_deflection = -elevator_deflection # elevator down is positive
        return elevator_deflection
    
    def aileron_controller(self, target_heading, current_heading, current_roll, current_roll_rate, dt):
        error = target_heading - current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        target_heading = error
        current_heading = 0
        roll_command = self.fw_heading.run_pid(target_heading, current_heading, dt)
        roll_rate_command = self.fw_roll.run_pid(roll_command, current_roll, dt)
        aileron_deflection = self.fw_roll_rate.run_pid(roll_rate_command, current_roll_rate, dt)
        return aileron_deflection
    
        # Step 2: Aileron controller (roll control)
    
    def rudder_controller(self, aileron, k_r2a):
        rudder_deflection = -k_r2a*aileron
        return rudder_deflection
    
    def run_fixed_wing_mode(self, current_state, target_airspeed, target_heading, target_altitude, dt):
        # Extract relevant state variables from current_states
        current_airspeed = current_state[0]  # Airspeed
        current_altitude = current_state[1]  # Altitude
        current_heading = current_state[2]  # Heading
        current_attitude = current_state[3:6]  # Attitude (roll, pitch, yaw)
        current_rates = current_state[6:9]  # Angular rates (roll_rate, pitch_rate, yaw_rate)

        # Compute throttle command using throttle_controller
        throttle_command = self.throttle_controller(target_airspeed, current_airspeed, dt)
        
        # Compute aileron deflection using aileron_controller
        aileron_deflection = self.aileron_controller(target_heading, current_heading, current_attitude[0], current_rates[0], dt)

        # Compute elevator deflection using elevator_controller
        elevator_deflection = self.elevator_controller(target_altitude, current_altitude, current_attitude[1], current_rates[1], dt)

        # Rudder deflection can be set to zero or computed if needed
        rudder_deflection = self.rudder_controller(aileron_deflection, 0.7)

        self.throttle[0:3] = np.zeros(3)  # No quadrotor throttle in fixed-wing mode
        # Return throttle command and a numpy array of control surface PWM values
        return throttle_command, np.array([aileron_deflection, elevator_deflection, rudder_deflection])

    