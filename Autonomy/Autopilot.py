# Autonomy/autopilot.py
import numpy as np
from Autonomy.Path_Planning import WaypointNavigator
from Autonomy.Controller import ControllerManager
from Global.Utils import linear_scale


class Autopilot_class:
    """
    Main Autopilot class for computing actuator commands based on flight mode, navigation goals, and current state.
    """

    def __init__(self, waypoints, dt):
        """
        Args:
            waypoints (list): List of 3D waypoints.
            dt (float): Control loop timestep.
        """
        self.dt = dt
        self.mode = "FW"  # "FW", "QUAD", or "TRANSITION"

        self.navigator = WaypointNavigator(waypoints)
        self.controller_mgr = ControllerManager(self.dt)

        # Final actuator outputs
        self.motor_pwm = np.zeros(5)
        self.ctrl_surface_pwm = np.zeros(3)

    def run(self, current_state):
        """
        Run the autopilot loop for one timestep.

        Args:
            current_state (np.ndarray): 12-element state vector [pos, vel, att, rates].

        Returns:
            tuple: (motor_pwm, ctrl_surface_pwm)
        """
        position = current_state[0:3]
        velocity = current_state[3:6]

        nav_output = self.navigator.update(position, velocity)
        if nav_output is None:
            return self.motor_pwm, self.ctrl_surface_pwm  # Hold or idle if no nav data

        nav_state = {
            "mode": self.mode,
            "position": position,
            "velocity": velocity,
            "target_pos": nav_output["target_pos"],          # Needed for QUAD
            "heading": nav_output["target_heading"],         # Needed for FW
            "airspeed": nav_output["target_airspeed"],       # Needed for FW
            "wp_type": nav_output.get("type", None)          # Optional
        }


        self.controller_mgr.fw.pids["altitude"].output_limits
        self.controller_mgr.fw.pids["airspeed"].output_limits
        self.controller_mgr.fw.pids["airspeed"].output_limits

        PWM_output = (1100, 2000)
        self.motor, self.ctrl_surface = self.controller_mgr.run(current_state, nav_state)

        self.motor_pwm[4] = linear_scale(self.motor[4], in_min=-10, in_max=10, out_min=PWM_output[0], out_max=PWM_output[1])
        self.ctrl_surface_pwm[0] = linear_scale(self.ctrl_surface[0], in_min=-30, in_max=30, out_min=PWM_output[0], out_max=PWM_output[1])
        self.ctrl_surface_pwm[1] = linear_scale(self.ctrl_surface[1], in_min=-30, in_max=30, out_min=PWM_output[0], out_max=PWM_output[1])
        self.ctrl_surface_pwm[2] = linear_scale(self.ctrl_surface[2], in_min=-30, in_max=30, out_min=PWM_output[0], out_max=PWM_output[1])
        return self.motor_pwm, self.ctrl_surface_pwm
