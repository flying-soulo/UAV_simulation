from vpython import *
import numpy as np
from GUI.data_transform import ned_to_eus
from GUI.environment import Environment
from GUI.aircraft import Aircraft

def make_sliders(scene, callback):
    labels = [
        ("North (m)", -1000, 1000, 0, "north"),
        ("East (m)", -1000, 1000, 0, "east"),
        ("Down (m)", -1000, 1000, 0, "down"),
        ("Roll (°)", -90, 90, 0, "roll"),
        ("Pitch (°)", -90, 90, 0, "pitch"),
        ("Yaw (°)", -180, 180, 0, "yaw"),
    ]
    sliders = {}
    scene.append_to_caption("=== UAV State Control ===\n")
    for label, mn, mx, init, key in labels:
        scene.append_to_caption(f"{label}: ")
        sliders[key] = slider(min=mn, max=mx, value=init, step=1, bind=callback, length=500)
        scene.append_to_caption("\n")
    return sliders

class UAVRenderer:
    def __init__(self, scene, manual_control=False):
        self.scene = scene
        self.scene.select()

        self.scene.title = "UAV 6DOF View (EUS)"
        self.scene.background = vector(0.53, 0.81, 0.92)
        self.scene.autoscale = False
        self.scene.lights = []
        distant_light(direction=vector(0.5, 0.5, -0.5), color=color.white)
        distant_light(direction=vector(-0.5, -0.5, 0.5), color=color.white)

        self.telemetry_label = wtext(text="Telemetry Initialized...\n")

        self.env = Environment(self.scene)
        self.env.build()
        self.uav = Aircraft()
        self.trail = curve(color=color.red, radius=0.2)

        self.cam_distance = 50.0
        self.cam_height = 25.0
        self.zoom_sensitivity = 5.0
        self.scene.bind("keydown", self.on_key_down)

        self.manual_control = manual_control

        if self.manual_control:
            self.sliders = make_sliders(self.scene, self.on_slider)
            self.update_from_sliders()
        else:
            self.sliders = None

    def on_key_down(self, evt):
        key = evt.key
        if key == "-":
            self.cam_distance = max(5.0, self.cam_distance - self.zoom_sensitivity)
            self.cam_height = max(2.0, self.cam_height - 0.5 * self.zoom_sensitivity)
        elif key == "=":
            self.cam_distance = min(200.0, self.cam_distance + self.zoom_sensitivity)
            self.cam_height = min(100.0, self.cam_height + 0.5 * self.zoom_sensitivity)

    def update_from_state(self, state):
        pos_ned = state[0:3]
        vel_body = state[3:6]
        orient_ned_rad = state[6:9]
        rates_rad = state[9:12]

        orient_deg = np.degrees(orient_ned_rad)
        pos_eus, rot_eus = ned_to_eus(pos_ned, orient_deg)
        self.uav.set_pose(pos_eus, rot_eus)

        self.trail.append(pos=vector(*pos_eus))

        cam_offset = vector(0, self.cam_height, self.cam_distance)
        cam_pos = vector(*pos_eus) + cam_offset
        self.scene.camera.pos = cam_pos
        self.scene.camera.axis = vector(*pos_eus) - cam_pos
        self.scene.camera.up = vector(0, 1, 0)

        self.show_telemetry(state)

    def show_telemetry(self, state):
        pos_ned = state[0:3]
        vel_body = state[3:6]
        angles_rad = state[6:9]
        rates_rad = state[9:12]
        angles_deg = np.rad2deg(angles_rad)
        rates_dps = np.rad2deg(rates_rad)

        text = (
            "=== UAV Telemetry ===\n"
            f"North (m): {pos_ned[0]:.1f} | "
            f"East  (m): {pos_ned[1]:.1f} | "
            f"Down  (m): {pos_ned[2]:.1f} \n"
            f"U (m/s): {vel_body[0]:.2f} | "
            f"V (m/s): {vel_body[1]:.2f} | "
            f"W (m/s): {vel_body[2]:.2f}\n"
            f"Roll  (°): {angles_deg[0]:.2f} | "
            f"Pitch (°): {angles_deg[1]:.2f} | "
            f"Yaw   (°): {angles_deg[2]:.2f}\n"
            f"P (deg/s): {rates_dps[0]:.2f} | "
            f"Q (deg/s): {rates_dps[1]:.2f} | "
            f"R (deg/s): {rates_dps[2]:.2f}\n"
        )
        self.telemetry_label.text = text

    def update_from_sliders(self):
        if not self.manual_control or self.sliders is None:
            return

        n = self.sliders["north"].value
        e = self.sliders["east"].value
        d = self.sliders["down"].value
        r = self.sliders["roll"].value
        p = self.sliders["pitch"].value
        y = self.sliders["yaw"].value

        pos_eus, rot_eus = ned_to_eus(np.array([n, e, d]), np.array([y, p, r]))  # ZYX
        self.uav.set_pose(pos_eus, rot_eus)

        cam_offset = vector(0, self.cam_height, self.cam_distance)
        cam_pos = vector(*pos_eus) + cam_offset

        self.scene.camera.pos = cam_pos
        self.scene.camera.axis = vector(*pos_eus) - cam_pos
        self.scene.camera.up = vector(0, 1, 0)

        state = [n, e, d, 0, 0, 0, np.deg2rad(r), np.deg2rad(p), np.deg2rad(y), 0, 0, 0]
        self.show_telemetry(state)

    def on_slider(self, _):
        self.update_from_sliders()
