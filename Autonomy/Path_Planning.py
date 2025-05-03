import numpy as np


class WaypointNavigator:
    def __init__(self, waypoints, loop=False, l1_distance=30.0):
        """
        :param waypoints: List of waypoints, each as (x, y, z, airspeed, type)
        :param loop: Whether to loop the waypoint list
        :param l1_distance: Lookahead distance for L1 controller
        """
        self.waypoints = waypoints
        self.loop = loop
        self.l1_distance = l1_distance
        self.reached_threshold = 5.0  # meters
        self.current_index = 1 if len(waypoints) > 1 else 0
        self.last_position = None

    def get_current_waypoint(self):
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def get_previous_waypoint(self):
        if self.current_index == 0:
            return self.waypoints[0]
        return self.waypoints[self.current_index - 1]

    def distance_to_waypoint(self, waypoint, position):
        dx, dy, dz = waypoint[0] - position[0], waypoint[1] - position[1], waypoint[2] - position[2]
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def vector_norm(self, v):
        return np.sqrt(np.sum(np.square(v)))

    def compute_cross_track_error(self, prev_wp, next_wp, position):
        # Vector from previous to current WP
        wp_vector = np.array(next_wp[:2]) - np.array(prev_wp[:2])
        wp_vector_norm = self.vector_norm(wp_vector)
        if wp_vector_norm < 1e-6:
            return 0.0

        # Vector from previous WP to current position
        pos_vector = np.array(position[:2]) - np.array(prev_wp[:2])

        # Cross-track error is magnitude of component orthogonal to track
        track_unit = wp_vector / wp_vector_norm
        cross_track_vec = pos_vector - np.dot(pos_vector, track_unit) * track_unit
        return self.vector_norm(cross_track_vec)

    def should_advance_waypoint(self, prev_wp, next_wp, position):
        # Vector from previous to next WP
        track_vec = np.array(next_wp[:2]) - np.array(prev_wp[:2])
        to_pos_vec = np.array(position[:2]) - np.array(prev_wp[:2])

        # Check if projection of pos on track is beyond next_wp
        dot = np.dot(track_vec, to_pos_vec)
        if dot < 0:
            return False

        # If projection length > track length, then passed WP
        return dot > np.dot(track_vec, track_vec)

    def update_waypoint(self, position):
        if self.current_index >= len(self.waypoints):
            return

        prev_wp = self.get_previous_waypoint()
        curr_wp = self.get_current_waypoint()

        # Switch waypoint based on cross-track-aware logic
        if self.should_advance_waypoint(prev_wp, curr_wp, position):
            self.current_index += 1
            if self.loop:
                self.current_index %= len(self.waypoints)

    def get_track_heading(self, prev_wp, curr_wp):
        dx, dy = curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]
        return np.arctan2(dy, dx)

    def l1_guidance(self, position, velocity):
        if self.current_index == 0 or self.current_index >= len(self.waypoints):
            return None

        prev_wp = self.get_previous_waypoint()
        curr_wp = self.get_current_waypoint()

        return self.get_track_heading(prev_wp, curr_wp)

    def update(self, position, velocity):
        """
        Main external call: updates waypoint and returns navigation info.
        :returns: dict with target_pos, target_heading, target_airspeed, type
        """
        self.last_position = position
        self.update_waypoint(position)

        curr_wp = self.get_current_waypoint()
        if curr_wp is None:
            return None

        prev_wp = self.get_previous_waypoint()
        heading = self.get_track_heading(prev_wp, curr_wp)

        return {
            "target_pos": np.array(curr_wp[:3]),
            "target_heading": heading,
            "target_airspeed": curr_wp[3],
            "type": curr_wp[4]
        }
