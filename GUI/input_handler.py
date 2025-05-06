"""
input_handler.py
Manages GUI buttons and updates to control structure.
"""

from vpython import *
import numpy as np


class GCSInput:
    def __init__(self):
        # Initialize control variables
        self.controls = {
            "thrust": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0,
        }

        self.command = None
        self.mode = "idle"
        self.home = (0.0, 0.0, 0.0)
        self.waypoint = [(0.0, 0.0, 0.0) for _ in range(3)]  # 3 waypoints

        # Store button references for coloring
        self.cmd_input = {}
        self.mode_input = {}

        self.create_controls()

        self.GCS_data = {
            "controls": None,
            "command": self.command,
            "mode": self.mode,
            "home": self.home,
            "waypoint": self.waypoint,
        }

    def create_controls(self):
        scene.append_to_caption("\n=== GCS COMMANDS ===\n")
        self.cmd_input["start"] = button(text="Start", bind=lambda _: self.set_command("start"))
        self.cmd_input["pause"] = button(text="Pause", bind=lambda _: self.set_command("pause"))
        self.cmd_input["stop"] = button(text="Stop", bind=lambda _: self.set_command("stop"))
        self.cmd_input["reset"] = button(text="Reset", bind=lambda _: self.set_command("reset"))

        scene.append_to_caption("\n=== MODES ===\n")
        self.mode_input["idle"] = button(text="IDLE", bind=lambda _: self.set_mode("idle"))
        self.mode_input["arm"] = button(text="ARM", bind=lambda _: self.set_mode("arm"))
        self.mode_input["takeoff"] = button(text="TAKEOFF", bind=lambda _: self.set_mode("takeoff"))
        self.mode_input["manual"] = button(text="MANUAL", bind=lambda _: self.set_mode("manual"))
        self.mode_input["land"] = button(text="LAND", bind=lambda _: self.set_mode("land"))

        scene.bind("keydown", self.keyboard_input)

        scene.append_to_caption("\n=== SET WAYPOINT ===\n")
        scene.append_to_caption("index | lattitude (radians) | longitude (radians) | altitude (m) \n")
        self.num_input = menu(bind=None, text="Select Waypoint", choices=["0", "1", "2", "3"])
        self.lat_input = winput(bind=None, text="")
        self.lon_input = winput(bind=None, text="")
        self.alt_input = winput(bind=None, text="")
        button(text="Set Waypoint", bind=self.set_waypoint_button)

        # Initialize default colors
        self.update_button_colors(None, self.cmd_input)
        self.update_button_colors("idle", self.mode_input)

    def update_button_colors(self, active_key, button_dict):
        for key, btn in button_dict.items():
            btn.background = color.green if key == active_key else color.gray(0.7)

    def set_command(self, cmd):
        self.command = cmd
        self.update_button_colors(cmd, self.cmd_input)

    def set_mode(self, mode):
        self.mode = mode
        self.update_button_colors(mode, self.mode_input)

    def keyboard_input(self, evt):
        key = evt.key
        if key == "w":
            self.controls["thrust"] += 1
        elif key == "s":
            self.controls["thrust"] -= 1
        elif key == "a":
            self.controls["yaw"] -= 1
        elif key == "d":
            self.controls["yaw"] += 1
        elif key == "up":
            self.controls["pitch"] += 1
        elif key == "down":
            self.controls["pitch"] -= 1
        elif key == "left":
            self.controls["roll"] -= 1
        elif key == "right":
            self.controls["roll"] += 1
        self.controls["thrust"] = np.clip(self.controls["thrust"], 0, 100)
        self.controls["pitch"] = np.clip(self.controls["pitch"], -10, 10)
        self.controls["roll"] = np.clip(self.controls["thrust"], -10, 10)
        self.controls["yaw"] = np.clip(self.controls["thrust"], -10, 10)

    def set_waypoint_button(self, _):
        try:
            num = int(self.num_input.selected)
            lat = float(self.lat_input.text)
            lon = float(self.lon_input.text)
            alt = float(self.alt_input.text)
            self.set_waypoint(num, lat, lon, alt)
        except ValueError:
            print("Invalid input. Please enter numeric values for latitude, longitude, and altitude.")


    def set_waypoint(self, num, lat, lon, alt):
        if num in [1, 2, 3]:
            self.waypoint[num - 1] = (lat, lon, alt)
        elif num == 0:
            self.home = (lat, lon, alt)
        else:
            print("Invalid waypoint number. Use 0 for home, 1-3 for waypoints.")

    def run(self):
        # Update dynamic values before returning
        self.GCS_data["controls"] = self.controls
        self.GCS_data["command"] = self.command
        self.GCS_data["mode"] = self.mode
        self.GCS_data["home"] = self.home
        self.GCS_data["waypoint"] = self.waypoint
        return self.GCS_data


def main():
    scene.title = "GCS Input Test"
    scene.width = 600
    scene.height = 400
    scene.center = vector(0, 0, 0)

    gcs = GCSInput()

    while True:
        rate(10)  # 10 Hz update rate

        data = gcs.run()

        # Display current data in console
        print("\r", end="")  # carriage return for inline overwrite
        print(
            f"MODE: {data['mode']:>8} | CMD: {str(data['command']):>6} | "
            f"CTRL: T:{data['controls']['thrust']:>2} R:{data['controls']['roll']:>2} "
            f"P:{data['controls']['pitch']:>2} Y:{data['controls']['yaw']:>2}",
            f" | HOME: {data['home']}",
            f" | WAYPOINTS: {data['waypoint'][0]}, {data['waypoint'][1]}, {data['waypoint'][2]}",
            end="",
        )


if __name__ == "__main__":
    main()
