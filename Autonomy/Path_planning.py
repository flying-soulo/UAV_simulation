import numpy as np
from Global.simdata import UAVState, Waypoint, GCSData, MissionTrack


class WaypointNavigator:
    """
    Handles navigation logic for advancing through mission waypoints
    based on UAV state and control mode.
    """

    def __init__(self):
        self.timer = 0  # Used for QD loiter check
        self.track = MissionTrack()

    def _distance_to_wp(self, state: UAVState, wp: Waypoint) -> float:
        """Computes 3D distance between UAV and a waypoint."""
        return float(np.linalg.norm([wp.x - state.x, wp.y - state.y, wp.z - state.z]))

    def should_advance(self, dist: float, mode: str) -> bool:
        """
        Determines if the UAV should move to the next waypoint based on:
        - distance to waypoint
        - vehicle mode
        """
        if mode == "FW":
            return dist < 120.0

        elif mode == "QD":
            self.timer = self.timer + 1 if dist < 5.0 else 0
            return self.timer > 100  # 1 second if running at 100Hz

        return False  # Don't advance in other modes

    def update(self, state: UAVState, gcs_data: GCSData) -> MissionTrack:
        """
        Updates the waypoint navigation logic:
        - Validates mission state
        - Computes distance to current waypoint
        - Advances to next waypoint if required
        - Updates mission track (prev/target)
        """
        mission = gcs_data.mission
        waypoints = mission.waypoints

        # Early exit: invalid mission or no waypoints
        if not waypoints:
            self.track = MissionTrack()  # reset just to be safe
            return self.track

        if not (0 <= mission.current_index < len(waypoints)):
            mission.current_index = 0  # Reset to first wp if corrupted index

        current_wp = waypoints[mission.current_index]
        dist = self._distance_to_wp(state, current_wp)

        if self.should_advance(dist, gcs_data.mode):
            next_idx = current_wp.next
            if 0 <= next_idx < len(waypoints):
                mission.previous_index = mission.current_index
                mission.current_index = next_idx
            else:
                mission.previous_index = mission.current_index
                mission.current_index = 0  # Restart loop

        # Ensure mission track is up to date
        mission.update_track()
        self.track = mission.track

        return self.track
