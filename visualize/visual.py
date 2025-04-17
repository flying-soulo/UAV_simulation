from vpython import *
import numpy as np

class UAVVisualizer:
    def __init__(self):
        # Simulation state
        self.state = [0, 0, 10, 0, 0, 0, 0, 0, 0]  # x, y, z, vx, vy, vz, roll, pitch, yaw

        # Dimensions
        self.env_limit = 10000
        self.sky_height = 10000

        # Aircraft geometry
        self.length_fuselage = 30
        self.width_fuselage = 3
        self.height_fuselage = 3
        self.wing_span = 35
        self.wing_chord = 4
        self.tail_span = 10
        self.tail_chord = 3
        self.nose_length = 4
        self.tail_height = 5

        # Main canvas (Fixed View)
        self.scene = canvas(title="Main View (Fixed)", width=1000, height=800,
                            center=vector(0, 0, 10),
                            background=vector(0.53, 0.81, 0.92), x=0, y=0)
        self.scene.camera.pos = vector(0, -200, 60)
        self.scene.camera.axis = vector(0, 200, -50)
        self.scene.up = vector(0, 0, 1)

        self.follow_view = canvas(title="Follow Camera View", width=500, height=400,
                                x=1000, y=0, background=vector(0.53, 0.81, 0.92))

        # Label Panel placed next to follow view
        self.label_panel = wtext(text="", height=16)
        self.label_box = canvas(title="Telemetry Panel", width=500, height=200,
                                x=1000, y=400, background=vector(1, 1, 1))

        # Create environment and aircraft
        self._create_environment()
        self._build_aircraft()
        self.update_visualization(self.state)

    def _create_environment(self):
        size = self.env_limit
        for scene_ref in [self.scene, self.follow_view]:
            scene_ref.select()
            ground = box(pos=vector(0, 0, -1), size=vector(size, size, 2), color=vector(0.2, 0.6, 0.2))
            runway = box(pos=vector(0, 0, 0.1), size=vector(size / 2, 40, 0.2), color=color.gray(0.5))
            sky = sphere(pos=vector(0, 0, 0), radius=self.sky_height, color=vector(0.53, 0.81, 0.92), opacity=0.2)

    def _build_aircraft(self):
        parts = []
        fuselage = box(pos=vector(0, 0, 0), size=vector(self.length_fuselage, self.width_fuselage, self.height_fuselage), color=color.red)
        nose = cone(pos=vector(self.length_fuselage / 2, 0, 0), axis=vector(self.nose_length, 0, 0), radius=1.5, color=color.orange)
        left_wing = box(pos=vector(0, -self.wing_span / 2, 0), size=vector(self.wing_chord, self.wing_span, 0.5), color=color.blue)
        right_wing = box(pos=vector(0, self.wing_span / 2, 0), size=vector(self.wing_chord, self.wing_span, 0.5), color=color.red)
        tail_plane = box(pos=vector(-self.length_fuselage / 2 + self.tail_chord / 2, 0, 0), size=vector(self.tail_chord, self.tail_span, 0.4), color=color.green)
        tail_fin = box(pos=vector(-self.length_fuselage / 2 + self.tail_chord / 2, 0, self.tail_height / 2), size=vector(self.tail_chord / 2, 0.4, self.tail_height), color=color.gray(0.6))

        parts += [fuselage, nose, left_wing, right_wing, tail_plane, tail_fin]
        self.aircraft = compound(parts)
        self.aircraft.pos = vector(0, 0, 10)

    def _euler_to_matrix(self, roll, pitch, yaw):
        # Standard aerospace (NED): Z down, X forward
        # VPython: Z up, X forward → we flip Z
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw),  np.cos(yaw), 0],
                    [0, 0, 1]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll),  np.cos(roll)]])
        R_ned = Rz @ Ry @ Rx

        # Flip Z to convert from NED to ENU (VPython style)
        flip_z = np.diag([1, 1, -1])
        return flip_z @ R_ned

    def update_visualization(self, state):
        pos_body = vector(state[0], state[1], -state[2])  # z is inverted
        phi, theta, psi = state[6], state[7], state[8]
        R_body = self._euler_to_matrix(phi, theta, psi)
        axis_vpy = vector(*R_body[:, 0])
        up_vpy = vector(*R_body[:, 2])

        self.aircraft.pos = pos_body
        self.aircraft.axis = axis_vpy * self.length_fuselage
        self.aircraft.up = up_vpy

        # Main camera fixed position but tracking aircraft
        self.scene.camera.pos = vector(0, -200, 60)
        self.scene.camera.axis = pos_body - self.scene.camera.pos

        self._update_main_label(state)
        self._update_follow_camera(pos_body, psi)

    def _update_follow_camera(self, position, yaw, distance=80, height=40):
        heading = vector(np.cos(yaw), np.sin(yaw), 0)
        cam_offset = -heading * distance + vector(0, 0, height)
        self.follow_view.camera.pos = position + cam_offset
        self.follow_view.camera.axis = position - self.follow_view.camera.pos
        self.follow_view.up = vector(0, 0, 1)

    def _update_main_label(self, state):
        roll, pitch, yaw = np.degrees(state[6]), np.degrees(state[7]), np.degrees(state[8])
        self.label_panel.text = (
            f"<b>UAV Data:</b><br>"
            f"Roll: {roll:.1f}°<br>"
            f"Pitch: {pitch:.1f}°<br>"
            f"Yaw: {yaw:.1f}°<br>"
            f"X: {state[0]:.1f} m<br>"
            f"Y: {state[1]:.1f} m<br>"
            f"Z: {-state[2]:.1f} m"
        )

    def _create_controls(self):
        def create_slider(title, index, min_val, max_val, step):
            wtext(text=f"{title} ")
            slider_obj = slider(min=min_val, max=max_val, value=self.state[index], step=step,
                                bind=lambda s: self._update_from_slider(s, index))
            wtext(text="\n")
            return slider_obj

        self.sliders = [
            create_slider("X (m):", 0, -100, 100, 1),
            create_slider("Y (m):", 1, -100, 100, 1),
            create_slider("Z (m):", 2, 0, 100, 1),
            create_slider("Roll (°):", 6, -np.pi/2, np.pi/2, np.radians(1)),
            create_slider("Pitch (°):", 7, -np.pi/2, np.pi/2, np.radians(1)),
            create_slider("Yaw (°):", 8, -np.pi, np.pi, np.radians(1))
        ]

    def _update_from_slider(self, slider_obj, index):
        self.state[index] = slider_obj.value
        self.update_visualization(self.state)


if __name__ == "__main__":
    visualizer = UAVVisualizer()
    visualizer._create_controls()
    
    while True:
        rate(10)
