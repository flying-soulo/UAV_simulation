# Autonomy/Controller/quad_controller.py
import numpy as np
from Autonomy.PID import PID_class
from Global.Utils import linear_scale


class QuadController:
    def __init__(self, dt):
        self.dt = dt
        self._init_pids()

    def _init_pids(self):
        # self.pids={}
        # for axis in ["x", "y", "z"]:
        #     self.pids[axis] : PID_class()
        #     self.pids[f"{axis}_vel"] : PID_class()

        # for axis in ['phi', 'theta', 'psi']:
        #     self.pids[axis] : PID_class()
        #     self.pids[f"{axis}_rate"] : PID_class()
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

    def run(self, current_state, nav_state):
        x, y, z = current_state[0:3]
        vx, vy, vz = current_state[3:6]
        phi, theta, psi = current_state[6:9]
        phi_rate, theta_rate, psi_rate = current_state[9:12]
        tx, ty, tz = nav_state["target_pos"]
        target_heading = nav_state["target_heading"]

        # x axis controller
        target_x_vel = self.pids["x"].run_pid(tx, x, self.dt)
        target_x_accel = self.pids["x_vel"].run_pid(target_x_vel, vx, self.dt)

        # y axis controller
        target_y_vel = self.pids["y"].run_pid(ty, y, self.dt)
        target_y_accel = self.pids["y_vel"].run_pid(target_y_vel, vy, self.dt)

        # Convert acceleration to desired angles using desired force and yaw
        AX = target_x_accel * np.cos(psi) + target_y_accel * np.sin(psi)
        AY = -target_x_accel * np.sin(psi) + target_y_accel * np.cos(psi)

        desired_theta = -AX / 9.81
        desired_phi = (np.cos(theta) * AY) / 9.81

        desired_theta = np.clip(desired_theta, ([np.deg2rad(-30), np.deg2rad(30)]))
        desired_phi = np.clip(desired_phi, ([np.deg2rad(-30), np.deg2rad(30)]))

        # roll controller
        desired_phi_rate = self.pids["phi"].run_pid(desired_phi, phi, self.dt)
        roll_output = self.pids["phi_rate"].run_pid(desired_phi_rate, phi_rate, self.dt)

        # pitch controller
        desired_theta_rate = self.pids["theta"].run_pid(desired_theta, theta, self.dt)
        pitch_output = self.pids["theta_rate"].run_pid(desired_theta_rate, theta_rate, self.dt)

        # yaw controller
        desired_psi_rate = self.pids["psi"].run_pid(target_heading, psi, self.dt)
        yaw_output = self.pids["psi_rate"].run_pid(desired_psi_rate, psi_rate, self.dt)

        # altitude controller
        desired_z_vel = self.pids["z"].run_pid(tz, z, self.dt)
        altitude_output = self.pids["z_vel"].run_pid(desired_z_vel, vz, self.dt)

        altitude_output = linear_scale(altitude_output, -5, 5, -1, 1)
        roll_output = linear_scale(roll_output, -5, 5, -1, 1)
        pitch_output = linear_scale(pitch_output, -5, 5, -1, 1)
        yaw_output = linear_scale(yaw_output, -5, 5, -1, 1)

        return {
            "throttle": altitude_output,
            "roll": roll_output,
            "pitch": pitch_output,
            "yaw": yaw_output,
        }
