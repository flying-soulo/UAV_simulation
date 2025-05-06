"""
visual.py
Integrates UAV Renderer and GCS Input Handler into a single window with a single canvas.
"""

from vpython import *
import numpy as np
from renderer import UAVRenderer
from input_handler import GCSInput

class UAV_GCS_Integration:
    def __init__(self):
        # Create the main scene for visualization
        self.scene = canvas(
            title="UAV 6DOF View with GCS Controls",
            width=1200,
            height=800,
            background=vector(0.53, 0.81, 0.92),
            autoscale=False,
        )

        # Initialize UAV Renderer with manual control
        self.uav_renderer = UAVRenderer(manual_control=True)
        self.uav_renderer.scene = self.scene  # Ensure UAV Renderer uses the same scene

        # Initialize GCS Input Handler
        self.gcs_input = GCSInput()
        self.gcs_input.create_controls()  # Setup the buttons and inputs

        # Link the GCS data with UAV Renderer
        self.gcs_data = None

        # Start the update loop
        self.run()

    def run(self):
        while True:
            rate(10)  # 10 Hz update rate

            # Get the latest GCS data
            self.gcs_data = self.gcs_input.run()

            # Update the UAV Renderer with the control data from GCS
            self.update_uav_state()

    def update_uav_state(self):
        # Extract control values from GCS input
        controls = self.gcs_data["controls"]
        mode = self.gcs_data["mode"]
        command = self.gcs_data["command"]

        # Update UAV state based on GCS inputs (convert control values to state)
        # For simplicity, we're assuming some basic control mapping to UAV state
        state = [
            0, 0, 0,  # Position (NED coordinates)
            controls["thrust"],  # Velocity in body frame (dummy data)
            0, 0,  # Velocity in body frame
            np.deg2rad(controls["roll"]),  # Roll angle
            np.deg2rad(controls["pitch"]),  # Pitch angle
            np.deg2rad(controls["yaw"]),  # Yaw angle
            0, 0, 0  # Rates (angular velocities in body frame)
        ]

        # Update the UAV Renderer with the new state
        self.uav_renderer.update(state)

        # Optionally, display current mode and command in the console
        print(
            f"MODE: {mode} | CMD: {command} | "
            f"CTRL: Thrust: {controls['thrust']} Roll: {controls['roll']} "
            f"Pitch: {controls['pitch']} Yaw: {controls['yaw']}"
        )


if __name__ == "__main__":
    # Initialize the integration and start the process
    uav_gcs = UAV_GCS_Integration()

# """

# UAV 6DOF Visualizer using VPython
# This script creates a 3D visualization of a UAV states from NED states converting to Vptyhon frame.
# """

# from vpython import *
# import numpy as np
# import time
# from data_transform import ned_to_eus
# from environment import Environment
# from aircraft import Aircraft

# def make_sliders(scene, callback):
#     labels = [
#         ("North (m)", -1000, 1000, 0, "north"),
#         ("East (m)", -1000, 1000, 0, "east"),
#         ("Down (m)", -1000, 1000, 0, "down"),
#         ("Roll (°)", -90, 90, 0, "roll"),
#         ("Pitch (°)", -90, 90, 0, "pitch"),
#         ("Yaw (°)", -180, 180, 0, "yaw"),
#     ]
#     sliders = {}
#     scene.append_to_caption("=== UAV State Control ===\n")
#     for label, mn, mx, init, key in labels:
#         scene.append_to_caption(f"{label}: ")
#         sliders[key] = slider(min=mn, max=mx, value=init, step=1, bind=callback, length=500)
#         scene.append_to_caption("\n")
#     return sliders

# class UAVVisualizer:
#     def __init__(self, manual_control=False):
#         self.scene = canvas(
#             title="UAV 6DOF View (EUS)",
#             width=1200,
#             height=800,
#             background=vector(0.53, 0.81, 0.92),
#             autoscale=False,
#         )
#         self.scene.lights = []
#         distant_light(direction=vector(0.5, 0.5, -0.5), color=color.white)
#         distant_light(direction=vector(-0.5, -0.5, 0.5), color=color.white)

#         self.telemetry_scene = canvas(
#             title="Telemetry",
#             width=400,
#             height=300,
#             background=color.white,
#         )
#         self.telemetry_scene.select()
#         self.telemetry_text = wtext(text="Waiting for data...\n")


#         self.scene.select()
#         self.env = Environment(self.scene)
#         self.env.build()
#         self.uav = Aircraft()
#         self.trail = curve(color=color.red, radius=0.2)


#         self.cam_distance = 50.0
#         self.cam_height = 25.0
#         self.zoom_sensitivity = 5.0
#         self.scene.bind("keydown", self.on_key_down)

#         self.states = np.zeros(12)
#         self.manual_control = manual_control

#         if self.manual_control:
#             self.sliders = make_sliders(self.scene, self.on_slider)
#             self.update_from_sliders()
#         else:
#             self.sliders = None

#     def on_slider(self, _):
#         self.update_from_sliders()

#     def on_key_down(self, evt):
#         key = evt.key
#         if key == "w":
#             self.cam_distance = max(5.0, self.cam_distance - self.zoom_sensitivity)
#             self.cam_height = max(2.0, self.cam_height - 0.5 * self.zoom_sensitivity)
#         elif key == "s":
#             self.cam_distance = min(200.0, self.cam_distance + self.zoom_sensitivity)
#             self.cam_height = min(100.0, self.cam_height + 0.5 * self.zoom_sensitivity)

#     def update_from_sliders(self):
#         if not self.manual_control or self.sliders is None:
#             return

#         n = self.sliders["north"].value
#         e = self.sliders["east"].value
#         d = self.sliders["down"].value
#         r = self.sliders["roll"].value
#         p = self.sliders["pitch"].value
#         y = self.sliders["yaw"].value

#         pos_eus, rot_eus = ned_to_eus(np.array([n, e, d]), np.array([y, p, r]))  # ZYX
#         self.uav.set_pose(pos_eus, rot_eus)

#         cam_offset = vector(0, self.cam_height, self.cam_distance)
#         cam_pos = vector(*pos_eus) + cam_offset

#         self.scene.camera.pos = cam_pos
#         self.scene.camera.axis = vector(*pos_eus) - cam_pos
#         self.scene.camera.up = vector(0, 1, 0)
#         # Optional: self.scene.camera.up = vector(*rot_eus.apply([0, 1, 0]))

#         states = [n, e, d, 0, 0, 0, np.deg2rad(r), np.deg2rad(p), np.deg2rad(y), 0, 0, 0]
#         self.telemetry(states)

#     def update_from_states(self, states):
#         self.states = states
#         pos_ned = states[0:3]
#         orient_ned_rad = states[6:9]
#         orient_ned_deg = np.degrees(orient_ned_rad)

#         pos_eus, rot_eus = ned_to_eus(pos_ned, orient_ned_deg)
#         self.uav.set_pose(pos_eus, rot_eus)

#         # Update trail
#         self.trail.append(pos=vector(*pos_eus))

#         cam_offset = vector(0, self.cam_height, self.cam_distance)
#         cam_pos = vector(*pos_eus) + cam_offset

#         self.scene.camera.pos = cam_pos
#         self.scene.camera.axis = vector(*pos_eus) - cam_pos
#         self.scene.camera.up = vector(0, 1, 0)

#         self.telemetry(self.states)


#     def telemetry(self, states):
#         pos_ned = states[0:3]
#         vel_body = states[3:6]
#         angles_rad = states[6:9]
#         rates_rad = states[9:12]

#         angles_deg = np.rad2deg(angles_rad)
#         rates_dps = np.rad2deg(rates_rad)

#         text = (
#             "=== UAV Telemetry ===\n"
#             f"North (m): {pos_ned[0]:.1f}\n"
#             f"East  (m): {pos_ned[1]:.1f}\n"
#             f"Down  (m): {pos_ned[2]:.1f}\n"
#             f"U (m/s): {vel_body[0]:.2f}\n"
#             f"V (m/s): {vel_body[1]:.2f}\n"
#             f"W (m/s): {vel_body[2]:.2f}\n"
#             f"Roll  (°): {angles_deg[0]:.2f}\n"
#             f"Pitch (°): {angles_deg[1]:.2f}\n"
#             f"Yaw   (°): {angles_deg[2]:.2f}\n"
#             f"P (deg/s): {rates_dps[0]:.2f}\n"
#             f"Q (deg/s): {rates_dps[1]:.2f}\n"
#             f"R (deg/s): {rates_dps[2]:.2f}\n"
#         )
#         self.telemetry_label.text = text


# # ---- Entry point ----
# if __name__ == "__main__":
#     viz = UAVVisualizer(manual_control=True)
#     while True:
#         rate(10)
