"""
input_handler.py
Manages GUI buttons and updates to control structure using dataclasses.
"""

from vpython import *  # type: ignore
import numpy as np
from Global.simdata import GCSData_class, Waypoint_class, Waypoint_data_class, Radio_data_class, Quad_controls


class GCSInput:
    def __init__(self, scene):
        self.scene = scene
        self.scene.select()

        self.controls: Quad_controls = Quad_controls()

        self.waypoint_data = Waypoint_data_class(
            home=Waypoint_class(0, 0, 0, 0, "", "", 0),
            waypoints = [Waypoint_class(0, 0, 0, 0, "", "", 1)],
            loop=False
        )

        self.GCS_data: GCSData_class = GCSData_class()

        self.cmd_buttons = {}
        self.mode_input = {}
        self.mode = "idle"

        self.create_controls()
        self.scene.bind("keydown", self.keyboard_input)

    def create_controls(self):
        self.scene.append_to_caption("\n=== GCS COMMANDS ===\n")
        self.cmd_buttons["start"] = button(text="Start", bind=lambda _: self.set_command("start"))
        self.cmd_buttons["pause"] = button(text="Pause", bind=lambda _: self.set_command("pause"))
        self.cmd_buttons["reset"] = button(text="Reset", bind=lambda _: self.set_command("reset"))
        self.cmd_buttons["stop"] = button(text="Stop", bind=lambda _: self.set_command("stop"))

        self.scene.append_to_caption("\n=== MODES ===\n")
        for m in ["idle", "arm", "takeoff", "manual", "land"]:
            self.mode_input[m] = button(text=m.upper(), bind=lambda _, mode=m: self.set_mode(mode))

        self.scene.append_to_caption("\n=== SET WAYPOINT ===\n")
        self.scene.append_to_caption("index | x | y | z\n")
        self.num_input = menu(text="Select Waypoint", choices=[0, 1, 2, 3], bind = None)
        self.lat_input = winput(bind = None)
        self.lon_input = winput(bind = None)
        self.alt_input = winput(bind = None)
        self.set_wp_button = button(text="Set Waypoint", bind=self.set_waypoint_button)

        self.scene.append_to_caption("\n=== Waypoints ===\n")
        self.home_text = wtext(text=f"Home: {self.waypoint_data.home}\n")
        self.wp_text = {
            k: wtext(text=f"Waypoint {k}: {v}\n")
            for k, v in enumerate(self.waypoint_data.waypoints)
        }

        self.update_button_colors(None, self.cmd_buttons)
        self.update_button_colors("idle", self.mode_input)


    def set_command(self, cmd):
        self.GCS_data.sim_command = cmd
        self.update_button_colors(cmd, self.cmd_buttons)

    def update_button_colors(self, active_key, button_dict):
        for key, btn in button_dict.items():
            btn.background = color.green if key == active_key else color.gray(0.7)


    def set_mode(self, mode):
        self.mode = mode
        self.update_button_colors(mode, self.mode_input)

    def keyboard_input(self, evt):
        if self.mode == "manual":
            k = evt.key
            if k == "w":
                self.controls.throttle += 1
            elif k == "s":
                self.controls.throttle-= 1
            elif k == "a":
                self.controls.yaw -= 1
            elif k == "d":
                self.controls.yaw += 1
            elif k == "up":
                self.controls.pitch += 1
            elif k == "down":
                self.controls.pitch -= 1
            elif k == "left":
                self.controls.roll -= 1
            elif k == "right":
                self.controls.roll += 1

        self.controls.throttle = np.clip(self.controls.throttle, -100, 100)
        self.controls.roll = np.clip(self.controls.roll, -10, 10)
        self.controls.pitch = np.clip(self.controls.pitch, -10, 10)
        self.controls.yaw = np.clip(self.controls.yaw, -10, 10)

    def set_waypoint_button(self, _):
        try:
            x = float(self.lat_input.text)
            y = float(self.lon_input.text)
            z = float(self.alt_input.text)
            if self.num_input.selected is not None:
                idx = int(self.num_input.selected)
                self.set_waypoint(idx, x, y, z)
                self.set_wp_button.color = color.green
                rate(1)
                self.set_wp_button.color = color.white
            else:
                raise ValueError

        except ValueError:
            self.set_wp_button.color = color.red
            rate(1)
            self.set_wp_button.color = color.white
        finally:
            self.scene.select()

    def set_waypoint(self, idx, x, y, z):
        wp = Waypoint_class(x=x, y=y, z=z, heading=0.0, action="", mode="", next=0)
        if idx == -1:
            self.waypoint_data.home = wp
            self.home_text.text = f"Home: {wp}\n"
        else:
            self.waypoint_data.waypoints[idx] = wp
            self.wp_text[idx].text = f"Waypoint {idx}: {wp}\n"
        self.GCS_data.update_waypoint = True
        self.GCS_data.current_waypoint = idx

    def run(self):
        self.radio_data = Radio_data_class(
            radio_channel1=self.controls.throttle,
            radio_channel2=self.controls.yaw,
            radio_channel3=self.controls.pitch,
            radio_channel4=self.controls.roll
        )

        self.GCS_data.mode = self.mode
        self.GCS_data.state = ""
        self.GCS_data.command = ""
        self.GCS_data.radio_data = self.radio_data
        return self.GCS_data


def main():
    scene.title = "GCS Input Test"
    scene.width = 600
    scene.height = 400
    scene.center = vector(0, 0, 0)

    gcs = GCSInput(scene)

    while True:
        rate(20)
        data = gcs.run()

        print("\r", end="")
        print(
            f"MODE: {data.mode} | CMD: {data.command} | "
            f"T:{data.radio_data.radio_channel1:.1f} "
            f"R:{data.radio_data.radio_channel4:.1f} "
            f"P:{data.radio_data.radio_channel3:.1f} "
            f"Y:{data.radio_data.radio_channel2:.1f} | "
            f"HOME: ({data.waypoint_data.home.x:.1f}, {data.waypoint_data.home.y:.1f}, {data.waypoint_data.home.z:.1f})",
            end=""
        )


if __name__ == "__main__":
    main()
