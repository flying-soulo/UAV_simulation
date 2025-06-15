# Autonomy/autopilot.py
import numpy as np
from Autonomy.Path_Planning import WaypointNavigator
from Autonomy.Controller import ControllerManager
from Autonomy.Mixer import Mixer
from Global.simdata import GCSData_class, Actuator_class, UAVState_class, Waypoint_data_class

class UAVAutopilot:
    """
    Main Autopilot class for computing actuator commands based on flight mode, navigation goals, and current state.
    """

    def __init__(self, GCS_data:GCSData_class, dt:float):
        """
        Args:

            waypoints (list): List of 3D waypoints.
            dt (float): Autopilot loop timestep.
        """
        self.dt = dt

        # Navigation manager
        self.navigator = WaypointNavigator(GCS_data.waypoint_data)

        # controller
        self.controller_mgr = ControllerManager(self.dt)

        # mixer
        self.mixer = Mixer()

        # Mixer output
        self.controls = Actuator_class(0,0,0,0,0,0,0,0)


    def run(self, current_state: UAVState_class, GCSdata: GCSData_class):
        """Main funciton which calculated the controls outputs for the autopilot

        Returns:
            control output: outputs the controls values for the UAV
        """

        nav_target = self.navigator.update(current_state, mode= GCSdata.mode)

        mixer_input = self.controller_mgr.run(current_state, nav_target, mode = GCSdata.mode)

        self.output = self.mixer.run(mode= GCSdata.mode, mixerinput= mixer_input)

        return self.output
