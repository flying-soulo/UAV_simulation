"""
UAV Interface
This module provides a graphical user interface (GUI) for visualizing and controlling a UAV (Unmanned Aerial Vehicle).
Integrates UAV Renderer and GCS Input Handler into a single window with a single canvas.
"""

from vpython import *
import numpy as np
from .renderer import UAVRenderer
from .input_handler import GCSInput
from Global.simdata import GCSData_class

class UAVinterface:
    def __init__(self,manual_control=False):
        # Create the main scene for visualization
        self.scene = canvas(
            title="UAV 6DOF View with GCS Controls",
            width=700,
            height=500,
            background=vector(0.53, 0.81, 0.92),
            autoscale=False,
        )
        self.scene.select()
        # rate(1)  # Set the rate for the main loop

        # Initialize UAV Renderer with manual control
        self.GCS = GCSInput(scene=self.scene)
        self.visual = UAVRenderer(scene = self.scene, manual_control = manual_control)
        self.output : GCSData_class

    def run(self):
        self.output = self.GCS.run()
        return self.output

    def update_uav_visual(self, state):
        self.visual.update_from_state(state)


if __name__ == "__main__":
    viz = UAVinterface(manual_control=True)
    data = viz.run()
    print(f"\r", end="")
    print(data)
    while True:
        rate(25)


