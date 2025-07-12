import numpy as np
from Global.simdata import UAVState, Waypoint, GCSData, MissionTrack


class WaypointNavigator:
    def __init__(self):
        self.index: int = 0
        self.timer: int = 0
        self.track = MissionTrack()  # Initialize properly

    def _distance_to_wp(self, state: UAVState, wp: Waypoint) -> float:
        return float(np.linalg.norm([wp.x - state.x, wp.y - state.y, wp.z - state.z]))

    def should_advance(self, state: UAVState, wp: Waypoint, mode: str) -> bool:
        dist = self._distance_to_wp(state, wp)

        if mode == "FW":
            return dist < 120.0

        elif mode == "QD":
            if dist < 5.0:
                self.timer += 1
            else:
                self.timer = 0
            return self.timer > 100

        return False

    def update(self, state: UAVState, gcs_data: GCSData) -> MissionTrack:
        mission = gcs_data.mission
        waypoints = mission.waypoints

        # No valid mission
        if not waypoints or self.index >= len(waypoints):
            return self.track

        current_wp = waypoints[self.index]

        if self.should_advance(state, current_wp, gcs_data.mode):
            next_idx = current_wp.next
            if 0 <= next_idx < len(waypoints):
                self.index = next_idx
            else:
                # Restart mission or hold last wp
                self.index = 0

        # Update mission track with current & previous waypoints
        self.track.previous = waypoints[max(self.index - 1, 0)]
        self.track.target = waypoints[self.index]
        mission.current_index = self.index  # Update mission state
        GCSData.mission.update_track()
        return self.track
