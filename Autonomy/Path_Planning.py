import numpy as np


class WaypointNavigator:
    def __init__(self, waypoints):
        """
        Initialize the WaypointNavigator with a list of waypoints.
        :param waypoints: List of waypoints, each waypoint is a tuple (x, y, z, airspeed,type).
        """
        self.waypoints = waypoints
        self.current_index = 0

    def distance_to_waypoint(self, waypoint, current_position):
        """
        Calculate the Euclidean distance to the given waypoint.
        :param waypoint: Target waypoint as a tuple (x, y, z, airspeed, type).
        :param current_position: Current position as a tuple (x, y, z).
        :return: Distance to the waypoint.
        """
        x1, y1, z1 = current_position
        x2, y2, z2, *_ = waypoint
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def get_waypoint_heading(self, waypoint, current_position):
        """
        Calculate the heading to the given waypoint relative to the current position.
        :param waypoint: Target waypoint as a tuple (x, y, z, airspeed, type).
        :param current_position: Current position as a tuple (x, y, z).
        :return: Heading in radians.
        """
        x1, y1, _ = current_position
        x2, y2, *_ = waypoint
        return np.arctan2(y2 - y1, x2 - x1)

    def get_current_waypoint(self):
        """
        Get the current waypoint based on the current index.
        :return: The current waypoint as a tuple.
        """
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def update_waypoint(self):
        """
        Update the current waypoint index by incrementing it by 1.
        """
        if self.current_index < len(self.waypoints) - 1:
            self.current_index += 1


# Example usage:
if __name__ == "__main__":
    waypoints = [(100, 100, 50), (200, 200, 100), (300, 300, 150)]
    airspeed = 50  # m/s

    navigator = WaypointNavigator(waypoints)

    # Simulate navigation
    positions = [
        (90, 90, 45),
        (100, 100, 50),
        (190, 190, 95),
        (200, 200, 100),
        (290, 290, 145),
        (300, 300, 150),
    ]
    for pos in positions:
        navigator.update_position(pos)
        navigator.navigate()
