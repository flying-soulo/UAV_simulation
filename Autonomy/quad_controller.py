# Autonomy/Controller/quad_controller.py
import numpy as np
from Autonomy.PID import PID_class
from Global.utils import linear_scale
from Global.simdata import Mission_track_data, UAVState_class, Quad_controls, Quad_target

class QuadController:
    def __init__(self, dt):
        self.dt = dt
        self.output : Quad_controls = Quad_controls()
        self.target : Quad_target = Quad_target()
        self._init_pids()

    def _init_pids(self):
        self.pids = {
            "x": PID_class(),
            "y": PID_class(),
            "z": PID_class(),
            "x_vel": PID_class(),
            "y_vel": PID_class(),
            "z_vel": PID_class(),
            "phi": PID_class(),
            "theta": PID_class(),
            "psi": PID_class(),
            "phi_rate": PID_class(),
            "theta_rate": PID_class(),
            "psi_rate": PID_class(),
        }

        # x position controller
        self.pids["x"].update_gains(kp=1, kd=0, ki=0)
        self.pids["x"].set_output_limits(lower=-5, upper=5)

        self.pids["x_vel"].update_gains(kp=1, kd=0, ki=0)
        self.pids["x_vel"].set_output_limits(lower=-5, upper=5)
        self.pids["x_vel"].set_integral_limits(lower=-3, upper=3)

        # y position controller
        self.pids["y"].update_gains(kp=1, kd=0, ki=0)
        self.pids["y"].set_output_limits(lower=-5, upper=5)
        self.pids["y_vel"].update_gains(kp=1, kd=0, ki=0)
        self.pids["y_vel"].set_output_limits(lower=-5, upper=5)
        self.pids["y_vel"].set_integral_limits(lower=-3, upper=3)

        # z position controller
        self.pids["z"].update_gains(kp=1, kd=0, ki=0)
        self.pids["z"].set_output_limits(lower=-5, upper=5)
        self.pids["z_vel"].update_gains(kp=1, kd=0, ki=0)
        self.pids["z_vel"].set_output_limits(lower=-5, upper=5)
        self.pids["z_vel"].set_integral_limits(lower=-3, upper=3)

        # roll controller
        self.pids["phi"].update_gains(kp=1, kd=0, ki=0)
        self.pids["phi"].set_output_limits(lower=-5, upper=5)
        self.pids["phi_rate"].update_gains(kp=1, kd=0, ki=0)
        self.pids["phi_rate"].set_output_limits(lower=-5, upper=5)
        self.pids["phi_rate"].set_integral_limits(lower=-3, upper=3)

        # pitch controller
        self.pids["theta"].update_gains(kp=1, kd=0, ki=0)
        self.pids["theta"].set_output_limits(lower=-5, upper=5)
        self.pids["theta_rate"].update_gains(kp=1, kd=0, ki=0)
        self.pids["theta_rate"].set_output_limits(lower=-5, upper=5)
        self.pids["theta_rate"].set_integral_limits(lower=-3, upper=3)

        # yaw controller
        self.pids["psi"].update_gains(kp=1, kd=0, ki=0)
        self.pids["psi"].set_output_limits(lower=-5, upper=5)
        self.pids["psi_rate"].update_gains(kp=1, kd=0, ki=0)
        self.pids["psi_rate"].set_output_limits(lower=-5, upper=5)
        self.pids["psi_rate"].set_integral_limits(lower=-3, upper=3)

    def run(self, current_state: UAVState_class , Mission_data: Mission_track_data):
        self.target.x =Mission_data.curr_wp.x
        self.target.y = Mission_data.curr_wp.y
        self.target.altitude = Mission_data.curr_wp.z
        self.target.heading = Mission_data.curr_wp.heading
        # x axis controller
        target_x_vel = self.pids["x"].run_pid(self.target.x, current_state.x, self.dt)
        target_x_accel = self.pids["x_vel"].run_pid(target_x_vel, current_state.x_vel, self.dt)

        # y axis controller
        target_y_vel = self.pids["y"].run_pid(self.target.y, current_state.y, self.dt)
        target_y_accel = self.pids["y_vel"].run_pid(target_y_vel, current_state.y_vel, self.dt)

        # Convert acceleration to desired angles using desired force and yaw
        AX = target_x_accel * np.cos(current_state.psi) + target_y_accel * np.sin(current_state.psi)
        AY = -target_x_accel * np.sin(current_state.psi) + target_y_accel * np.cos(current_state.psi)

        desired_theta = -AX / 9.81
        desired_phi = (np.cos(current_state.theta) * AY) / 9.81

        desired_theta = np.clip(desired_theta.astype(float), a_min=np.deg2rad(-30), a_max=np.deg2rad(30))
        desired_phi   = np.clip(desired_phi.astype(float),   a_min=np.deg2rad(-30), a_max=np.deg2rad(30))

        # roll controller
        desired_phi_rate = self.pids["phi"].run_pid(desired_phi, current_state.phi, self.dt)
        roll_output = self.pids["phi_rate"].run_pid(desired_phi_rate, current_state.phi_rate, self.dt)

        # pitch controller
        desired_theta_rate = self.pids["theta"].run_pid(desired_theta, current_state.theta, self.dt)
        pitch_output = self.pids["theta_rate"].run_pid(desired_theta_rate, current_state.theta_rate, self.dt)

        # yaw controller
        desired_psi_rate = self.pids["psi"].run_pid(self.target.heading, current_state.psi, self.dt)
        yaw_output = self.pids["psi_rate"].run_pid(desired_psi_rate, current_state.psi_rate, self.dt)

        # altitude controller
        desired_z_vel = self.pids["z"].run_pid(self.target.altitude, current_state.z, self.dt)
        altitude_output = self.pids["z_vel"].run_pid(desired_z_vel, current_state.z_vel, self.dt)

        self.output.throttle = linear_scale(altitude_output, -5, 5, -1, 1)
        self.output.roll = linear_scale(roll_output, -5, 5, -1, 1)
        self.output.pitch = linear_scale(pitch_output, -5, 5, -1, 1)
        self.output.yaw = linear_scale(yaw_output, -5, 5, -1, 1)

        return self.output
