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


# import numpy as np
# from Global.simdata import (
#     Waypoint_data_class,
#     Waypoint_class,
#     Mission_track_data,
#     UAVState_class,
# )


# class WaypointNavigator:
#     def __init__(self, waypoint_data: Waypoint_data_class):
#         self.waypoint_data = waypoint_data
#         self.current_index = 0
#         self.previous_index = 0
#         self.next_index = 0
#         self.output: Mission_track_data = Mission_track_data()
#         self.quad_timer = 0

#     def upload_waypoints(self, waypoint_data: Waypoint_data_class):
#         self.waypoint_data = waypoint_data

#     def should_advance_waypoint(self, current_state: UAVState_class, mode: str):
#         curr_wp = self.waypoint_data.waypoints[self.current_index]
#         dist = float(
#             np.linalg.norm(
#                 [
#                     curr_wp.x - current_state.x,
#                     curr_wp.y - current_state.y,
#                     curr_wp.z - current_state.z,
#                 ]
#             )
#         )
#         if mode == "FW":
#             return dist < 120
#         elif mode == "QD":
#             if dist < 5:
#                 self.quad_timer += 1
#             elif dist > 5:
#                 self.quad_timer = 0

#             if self.quad_timer > 500:
#                 return True
#             else:
#                 return False

#     def get_track_heading(
#         self, prev_wp: Waypoint_class, curr_wp: Waypoint_class
#     ) -> float:
#         dx = curr_wp.x - prev_wp.x
#         dy = curr_wp.y - prev_wp.y
#         return np.arctan2(dy, dx)

#     def update(self, current_position: UAVState_class, mode: str) -> Mission_track_data:
#         if self.should_advance_waypoint(current_position, mode):
#             self.next_index = self.waypoint_data.waypoints[self.current_index].next

#             if self.next_index in [wp.next for wp in self.waypoint_data.waypoints]:
#                 self.previous_index = self.current_index
#                 self.current_index = self.next_index
#             else:
#                 self.previous_index = self.current_index
#                 self.current_index = -1

#             self.output.curr_wp = self.waypoint_data.waypoints[self.current_index]
#             self.output.prev_wp = self.waypoint_data.waypoints[self.current_index]

#         return self.output
