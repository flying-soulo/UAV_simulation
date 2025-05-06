# Autonomy/autopilot.py
import numpy as np
from Autonomy.Path_Planning import WaypointNavigator
from Autonomy.Controller import ControllerManager
from Global.Utils import linear_scale
from Autonomy.Mixer import Mixer


class UAVAutopilot:
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

        self.mixer = Mixer()

        self.control_output = [0] * 8

    def compute_control(self, state):
        motor_thrust, ctrl_srfc_deflection = self.autopilot.run(state)
        motor_thrust = np.clip(motor_thrust, self.thrust_min, self.thrust_max)
        ctrl_srfc_deflection = np.clip(
            ctrl_srfc_deflection, self.deflection_min, self.deflection_max
        )
        return motor_thrust, ctrl_srfc_deflection

    def run(self, current_state):
        position = current_state[0:3]
        velocity = current_state[3:6]

        nav_output = self.navigator.update(position, velocity)
        if nav_output is None:
            self.control_output = [0] * 8  # Hold or idle if no nav data
            return self.control_output

        nav_state = {
            "mode": self.mode,
            "target_pos": nav_output["target_pos"],  # Needed for QUAD
            "heading": nav_output["target_heading"],  # Needed for FW
            "airspeed": nav_output["target_airspeed"],  # Needed for FW
            "wp_type": nav_output.get("type", None),  # Optional
        }

        # compute control commands based on the current state and navigation state using the controller manager
        input_motor, input_ctrlsrfc = self.controller_mgr.run(current_state, nav_state)
        mixer_input = input_motor + input_ctrlsrfc
        self.control_output = self.mixer.run(mode= "QuadPlane", mixerinput=mixer_input)


        return self.control_output
