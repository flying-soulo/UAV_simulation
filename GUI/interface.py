"""
UAV Interface
This module provides a graphical user interface (GUI) for visualizing and controlling a UAV (Unmanned Aerial Vehicle).
Integrates UAV Renderer and GCS Input Handler into a single window with a single canvas.
"""

from vpython import *  # type: ignore
import numpy as np
from GUI.renderer import UAVRenderer
from GUI.input_handler import GCSInput
from Global.simdata import GCSData, Waypoint

class UAVinterface:
    def __init__(self, GCS_data:GCSData, manual_control:bool=False):
        # Create the main scene for visualization
        self.scene = canvas(
            title="UAV 6DOF View with GCS Controls",
            width=700,
            height=500,
            background=vector(0.53, 0.81, 0.92),
            autoscale=False,
        )
        self.scene.select()

        # Initialize UAV Renderer with manual control
        self.GCS = GCSInput(self.scene, GCS_data)
        self.visual = UAVRenderer(scene = self.scene, manual_control = manual_control)
        self.output : GCSData = GCSData()


    def run(self):
        self.output = self.GCS.run()
        return self.output

    def update_uav_visual(self, state):
        self.visual.update_from_state(state)


if __name__ == "__main__":
    viz = UAVinterface(GCS_data=GCSData(), manual_control=True)
    data = viz.run()
    print(f"\r", end="")
    print(data)
    while True:
        rate(25)


