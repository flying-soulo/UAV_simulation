"""
UAV Interface
This module provides a graphical user interface (GUI) for visualizing and controlling a UAV (Unmanned Aerial Vehicle).
Integrates UAV Renderer and GCS Input Handler into a single window with a single canvas.
"""

from vpython import *
import numpy as np
from GUI.renderer import UAVRenderer
from GUI.input_handler import GCSInput

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
        self.visual = UAVRenderer(scene=self.scene, manual_control=manual_control)

    def run(self):
        self.gcs_data = self.GCS.run()
        data = self.gcs_data
        return self.gcs_data

    def update_uav_visual(self, state):
        self.visual.update_from_state(state)


if __name__ == "__main__":
    # Uncomment the following lines to run the UAV visualizer independently
    viz = UAVinterface(manual_control=False)
    data = viz.run()
    print(f"\r", end="")
    print(data)
    while True:
        rate(25)


