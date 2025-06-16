import numpy as np
from Global.simdata import Waypoint_data_class, Waypoint_class, Mission_track_data, UAVState_class


class WaypointNavigator:
    def __init__(self, waypoint_data: Waypoint_data_class):
        self.waypoint_data = waypoint_data
        self.current_index = 0
        self.previous_index = 0
        self.next_index = 0
        self.output: Mission_track_data = Mission_track_data()

    def update_waypoint(self, waypoint_data: Waypoint_class, num: int):
        self.waypoint_data.waypoints[num] = waypoint_data

    def should_advance_waypoint(self, current_state: UAVState_class, mode: str):
        curr_wp = self.waypoint_data.waypoints[self.current_index]
        dist = float(
            np.linalg.norm(
                [
                    curr_wp.x - current_state.x,
                    curr_wp.y - current_state.y,
                    curr_wp.z - current_state.z,
                ]
            )
        )
        if mode == "FW":
            return dist < 120
        elif mode == "Quad":
            timer = 0
            if dist < 5:
                timer = timer + 1
            elif dist > 5:
                timer = 0

            if timer > 500:
                return True
            else:
                return False

    def get_track_heading(
        self, prev_wp: Waypoint_class, curr_wp: Waypoint_class
    ) -> float:
        dx = curr_wp.x - prev_wp.x
        dy = curr_wp.y - prev_wp.y
        return np.arctan2(dy, dx)

    def update(self, current_position: UAVState_class, mode: str) -> Mission_track_data:
        if self.should_advance_waypoint(current_position, mode):
            self.next_index = self.waypoint_data.waypoints[self.current_index].next

            if self.next_index in [wp.next for wp in self.waypoint_data.waypoints]:
                self.previous_index = self.current_index
                self.current_index = self.next_index
            else:
                self.previous_index = self.current_index
                self.current_index = -1  # End of mission

            self.output.curr_wp = self.waypoint_data.waypoints[self.current_index]
            self.output.prev_wp = self.waypoint_data.waypoints[self.current_index]

        return self.output
