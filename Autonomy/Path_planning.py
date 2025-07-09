import numpy as np
from Global.simdata import Mission_track_data, UAVState_class, Waypoint_data_class

class WaypointNavigator:
    def __init__(self):
        self.track: Mission_track_data = Mission_track_data()
        self.index: int = 0
        self.timer: int = 0

    def should_advance(self, state: UAVState_class, wp, mode: str) -> bool:
        dist = float(np.linalg.norm([
            wp.x - state.x,
            wp.y - state.y,
            wp.z - state.z
        ]))

        if mode == "FW":
            return dist < 120

        elif mode == "QD":
            if dist < 5:
                self.timer += 1
            else:
                self.timer = 0
            return self.timer > 100

        return False

    def update(self, state: UAVState_class, wp_data: Waypoint_data_class, mode: str) -> Mission_track_data:
        if self.index >= len(wp_data.waypoints):
            return self.track  # Mission finished or invalid

        current_wp = wp_data.waypoints[self.index]

        # Check whether to advance to the next waypoint
        if self.should_advance(state, current_wp, mode):
            next_idx = current_wp.next
            if 0 <= next_idx < len(wp_data.waypoints):
                self.index = next_idx

        # Update track
        self.track.curr_wp = wp_data.waypoints[self.index]
        prev_idx = max(self.index - 1, 0)
        self.track.prev_wp = wp_data.waypoints[prev_idx]

        return self.track
