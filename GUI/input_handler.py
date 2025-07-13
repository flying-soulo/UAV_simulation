"""
input_handler.py
Manages GUI buttons and updates to control structure using dataclasses.
"""

import logging
from vpython import *  # type: ignore
import numpy as np
from Global.simdata import GCSData, Waypoint, RCInput

# === LOGGING CONFIG ===
DEBUG_TO_CONSOLE = False  # Toggle between console or file logging

if DEBUG_TO_CONSOLE:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
else:
    logging.basicConfig(
        filename="logger/gcs_debug.log",
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )


class GCSInput:
    def __init__(self, scene: canvas, output: GCSData):
        self.scene = scene
        self.scene.select()

        self.output: GCSData = GCSData()
        self.radio: RCInput = RCInput()

        self.sim_cmd_buttons = {}
        self.AC_cmd_buttons = {}
        self.mode_buttons = {}
        self.waypoint_buttons = {}

        self.previous_mode: str = "AUTO"
        self.create_controls()
        self.scene.bind("keydown", self.radio_control_input)
        # GCS data initialization
        self.output.mission.home = Waypoint(x=1000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        self.output.mission.waypoints = [
            Waypoint(x=3000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=1),
            Waypoint(x=3000, y=3000, z=-1000, heading=0, action="reach", mode="Auto", next=2),
            Waypoint(x=-3000, y=3000, z=-1000, heading=0, action="reach", mode="Auto", next=3),
            Waypoint(x=-3000, y=-3000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        ]
        self.output.mission.current_index = 0
        self.output.mission.previous_index = 0


    def radio_control_input(self, evt: event_return):
        if self.output.mode == "MANUAL ":
            k = evt.key
            if k == "w":
                self.radio.throttle += 1
            elif k == "s":
                self.radio.throttle -= 1
            elif k == "a":
                self.radio.yaw -= 1
            elif k == "d":
                self.radio.yaw += 1
            elif k == "up":
                self.radio.pitch += 1
            elif k == "down":
                self.radio.pitch -= 1
            elif k == "left":
                self.radio.roll -= 1
            elif k == "right":
                self.radio.roll += 1
            elif k == "q":
                if self.radio.mode_switch == "FW_MANUAL":
                    self.radio.mode_switch = "QD_MANUAL"
                elif self.radio.mode_switch == "FW_MANUAL":
                    self.radio.mode_switch = "QD_MANUAL"

        self.radio.throttle = np.clip(self.radio.throttle, -100, 100)
        self.radio.roll = np.clip(self.radio.roll, -100, 100)
        self.radio.pitch = np.clip(self.radio.pitch, -100, 100)
        self.radio.yaw = np.clip(self.radio.yaw, -100, 100)

    def create_sim_command_inputs(self):
        self.scene.append_to_caption("\n              SIM COMMANDS              \n")
        for m in ["START", "PAUSE", "RESET", "STOP"]:
            self.sim_cmd_buttons[m] = button(
                text=m.upper(),
                bind=lambda _, m=m: (
                    setattr(self.output, "sim_command", m),
                    self.update_button_colors(m, self.sim_cmd_buttons),
                ),
            )
            self.scene.append_to_caption(" ")

    def create_gcs_command_inputs(self):
        self.scene.append_to_caption("\n         GCS COMMANDS                 \n")
        for command in ["ARM", "LAUNCH", "ABORT", "DISARM", "LAND"]:
            self.AC_cmd_buttons[command] = button(
                text=command.upper(),
                bind=lambda _, m=command: (
                    setattr(self.output, "sim_command", m),
                    self.update_button_colors(m, self.AC_cmd_buttons),
                ),
            )
            self.scene.append_to_caption(" ")

    def set_gcs_command(self, mode: str):

        self.update_button_colors(mode, self.mode_buttons)

    def create_mode_inputs(self):
        self.scene.append_to_caption("\n         MODES                 \n")
        for mode in ["AUTO", "QD ALTHOLD", "QD POSHOLD", "MANUAL"]:
            self.mode_buttons[mode] = button(
                text=mode.upper(),
                bind=lambda _, m=mode: (
                    setattr(self.output, "mode", m),
                    self.update_button_colors(m, self.mode_buttons),
                ),
            )
            self.scene.append_to_caption(" ")

    def create_waypoint_inputs(self):
        self.scene.append_to_caption("  \n")
        self.scene.append_to_caption("\n        SET WAYPOINT        \n")
        self.scene.append_to_caption("index: ")
        self.waypoint_index_input = menu(
            text="index", choices=["home", 0, 1, 2, 3, 4], bind=self.show_input_waypoint
        )

        self.scene.append_to_caption(" x (in m): ")
        self.waypoint_buttons["x"] = winput(bind=None)

        self.scene.append_to_caption(" y (in m): ")
        self.waypoint_buttons["y"] = winput(bind=None)

        self.scene.append_to_caption(" z (in m): ")
        self.waypoint_buttons["z"] = winput(bind=None)

        self.scene.append_to_caption(" heading (in deg): ")
        self.waypoint_buttons["heading"] = winput(bind=None)

        self.scene.append_to_caption(" Action: ")
        self.waypoint_buttons["action"] = menu(
            text=" Waypoint",
            choices=["reach", "hover", "loiter"],
            bind=self.show_input_waypoint,
        )

        self.scene.append_to_caption(" Mode: ")
        self.waypoint_buttons["mode"] = menu(
            text=" Waypoint",
            choices=["Auto", "Quad", "FW"],
            bind=self.show_input_waypoint,
        )

        self.scene.append_to_caption(" next ")
        self.waypoint_buttons["next"] = menu(
            text=" next: ", choices=["home", 0, 1, 2, 3, 4], bind=None
        )

        self.scene.append_to_caption(" ")
        self.set_wp_button = button(text="save Waypoint", bind=self.set_waypoint)

        self.scene.append_to_caption("  ")
        self.upload_waypoint_button = button(
            text="Upload waypoints",
            bind=lambda: setattr(self.output, "upload_waypoints", True),
        )

    def show_input_waypoint(self):
        wp = self.waypoint_index_input.selected

        if wp is None:
            return

        if wp == "home":
            wp_data = self.output.mission.home
        else:
            try:
                idx = int(wp)
                wp_data = self.output.mission.waypoints[idx]
            except (ValueError, IndexError):
                return  # Invalid input or index out of range

        self.waypoint_buttons["x"].text = str(wp_data.x)
        self.waypoint_buttons["y"].text = str(wp_data.y)
        self.waypoint_buttons["z"].text = str(wp_data.z)
        self.waypoint_buttons["heading"].text = str(wp_data.heading)
        self.waypoint_buttons["mode"].text = wp_data.mode
        self.waypoint_buttons["action"].text = wp_data.action
        self.waypoint_buttons["next"].text = wp_data.next

    def set_waypoint(self):
        try:
            wp_id = self.waypoint_index_input.selected
            if wp_id is None:
                raise ValueError("Invalid waypoint index")

            wp = Waypoint(
                x=float(self.waypoint_buttons["x"].text),
                y=float(self.waypoint_buttons["y"].text),
                z=float(self.waypoint_buttons["z"].text),
                heading=float(self.waypoint_buttons["heading"].text),
                mode=self.waypoint_buttons["mode"].text,
                action=self.waypoint_buttons["action"].text,
                next=self.waypoint_buttons["next"].text,
            )

            if wp_id == "home":
                self.output.mission.home = wp
            else:
                self.output.mission.waypoints[int(wp_id)] = wp

            self.set_wp_button.color = color.green
            logging.info(f"Waypoint '{wp_id}' saved: {wp}")
        except Exception as e:
            self.set_wp_button.color = color.red
            logging.error(f"Failed to save waypoint: {e}")
        finally:
            rate(1)
            self.set_wp_button.color = color.white
            self.scene.select()

    def update_button_colors(
        self, active_key, button_dict: dict, highlight=color.green
    ):
        for key, btn in button_dict.items():
            target_color = highlight if key == active_key else color.gray(0.7)
            if btn.background != target_color:
                btn.background = target_color

    def create_controls(self):
        self.create_sim_command_inputs()
        self.create_gcs_command_inputs()
        self.create_mode_inputs()
        self.create_waypoint_inputs()

    def run(self):
        self.output.rc = self.radio

        return self.output


# def main():
#     scene.title = "GCS Input Test"
#     scene.width = 600
#     scene.height = 400
#     scene.center = vector(0, 0, 0)

#     gcs = GCSInput(scene)

#     while True:
#         rate(20)
#         data = gcs.run()

#         logging.info(
#             f"MODE: {data.mode} | CMD: {data.command} | "
#             f"T:{data.radio.roll:.1f} "
#             f"R:{data.radio.yaw:.1f} "
#             f"P:{data.radio.throttle:.1f} "
#             f"Y:{data.radio.pitch:.1f} | "
#             f"HOME: ({data.mission_plan.home.x:.1f}, "
#             f"{data.mission_plan.home.y:.1f}, "
#             f"{data.mission_plan.home.z:.1f})"
#         )


# if __name__ == "__main__":
#     main()
