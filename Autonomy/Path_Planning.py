import numpy as np


class WaypointNavigator:
    def __init__(self, waypoints, loop=False, l1_distance=30.0):
        """
        :param waypoints: List of waypoints, each as (x, y, z, airspeed, type)
        :param loop: Whether to loop the waypoint list
        :param l1_distance: Lookahead distance for L1 controller
        """
        self.waypoints = waypoints
        self.current_index = 0
        self.loop = loop
        self.l1_distance = l1_distance
        self.reached_threshold = 5.0  # meters
        self.last_position = None

    def get_current_waypoint(self):
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def update_waypoint(self, position):
        wp = self.get_current_waypoint()
        if wp and self.distance_to_waypoint(wp, position) < self.reached_threshold:
            self.current_index += 1
            if self.loop:
                self.current_index %= len(self.waypoints)

    def distance_to_waypoint(self, waypoint, position):
        dx, dy, dz = waypoint[0] - position[0], waypoint[1] - position[1], waypoint[2] - position[2]
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def get_waypoint_heading(self, waypoint, position):
        dx, dy = waypoint[0] - position[0], waypoint[1] - position[1]
        return np.arctan2(dy, dx)

    def l1_guidance(self, position, velocity):
        wp = self.get_current_waypoint()
        if wp is None:
            return None
        return self.get_waypoint_heading(wp, position)

    def update(self, position, velocity):
        """
        Main external call: updates waypoint and returns navigation info.
        :returns: dict with target_pos, target_heading, target_airspeed, type
        """
        self.last_position = position
        self.update_waypoint(position)

        wp = self.get_current_waypoint()
        if wp is None:
            return None

        heading = self.get_waypoint_heading(wp, position)
        return {
            "target_pos": np.array(wp[:3]),
            "target_heading": heading,
            "target_airspeed": wp[3],
            "type": wp[4]
        }
