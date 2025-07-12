# Autonomy/autopilot.py
import numpy as np
from Autonomy.Path_planning import WaypointNavigator
from Autonomy.Controller import ControllerManager
from Autonomy.Mixer import Mixer
from Global.simdata import GCSData, ActuatorOutputs, UAVState, Waypoint
from Autonomy.FMM import Flight_Mode_manager

class UAVAutopilot:
    """
    Main Autopilot class for computing actuator commands based on flight mode, navigation goals, and current state.
    """

    def __init__(self, GCS_data:GCSData, dt:float):
        """
        Args:

            waypoints (list): List of 3D waypoints.
            dt (float): Autopilot loop timestep.
        """
        self.dt = dt

        # Flight mode manager
        self.FMM = Flight_Mode_manager(GCS_data)

        # controller
        self.controller_mgr = ControllerManager(self.dt)

        # mixer
        self.mixer = Mixer()


    def run(self, current_state: UAVState, GCSdata: GCSData):
        """
        Main funciton which calculated the Actuator outputs for the autopilot
        """

        nav_target, controller_flags = self.FMM.run(GCSdata, current_state)

        mixer_input = self.controller_mgr.run(current_state, nav_target, controller_flags)

        self.output = self.mixer.run(GCSdata.mode, mixer_input)

        return self.output
