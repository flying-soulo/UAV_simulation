import numpy as np
from Global.simdata import Waypoint_class, FW_target, Mission_track_data

class FW_guidance:
    def __init__(self, min_L1_dist=20.0, L1_ratio=1.5, default_airspeed=30.0):
        self.min_L1_dist = min_L1_dist
        self.L1_ratio = L1_ratio
        self.default_airspeed = default_airspeed  # m/s
        self.output: FW_target = FW_target()

    def vector_norm(self, v: np.ndarray) -> float:
        return float(np.linalg.norm(v))

    def get_path_unit_vector(self, start: Waypoint_class, end: Waypoint_class):
        vec = np.array([end.x - start.x, end.y - start.y])
        norm = self.vector_norm(vec)
        return vec / norm if norm > 1e-6 else np.array([1.0, 0.0])

    def compute_cross_track_error(self, prev_wp: Waypoint_class, next_wp: Waypoint_class, pos: np.ndarray) -> float:
        path_vec = np.array([next_wp.x - prev_wp.x, next_wp.y - prev_wp.y])
        pos_vec = np.array([pos[0] - prev_wp.x, pos[1] - prev_wp.y])
        path_unit = path_vec / self.vector_norm(path_vec)
        proj = np.dot(pos_vec, path_unit)
        closest_point = np.array([prev_wp.x, prev_wp.y]) + proj * path_unit
        error_vec = np.array([pos[0], pos[1]]) - closest_point
        return self.vector_norm(error_vec)

    def compute_guidance(self, waypoint_data:Mission_track_data, position: np.ndarray, velocity: np.ndarray)-> FW_target:
        prev_wp = waypoint_data.curr_wp
        curr_wp = waypoint_data.prev_wp

        # --- Path geometry ---
        path_vec = np.array([curr_wp.x - prev_wp.x, curr_wp.y - prev_wp.y])
        path_norm = self.vector_norm(path_vec)
        if path_norm < 1e-6:
            return FW_target(roll = 0, altitude = curr_wp.z, airspeed = self.default_airspeed)

        path_unit = path_vec / path_norm
        pos_vec = np.array([position[0] - prev_wp.x, position[1] - prev_wp.y])
        along_track_dist = np.dot(pos_vec, path_unit)
        closest_point = np.array([prev_wp.x, prev_wp.y]) + along_track_dist * path_unit
        L1_vec = (along_track_dist + self.get_L1_distance(velocity)) * path_unit - pos_vec

        # --- Angle between velocity and L1 vector ---
        eta = np.arctan2(L1_vec[1], L1_vec[0]) - np.arctan2(velocity[1], velocity[0])
        eta = np.arctan2(np.sin(eta), np.cos(eta))  # wrap to [-π, π]

        # --- Lateral acceleration command ---
        groundspeed = self.vector_norm(velocity)
        L1_dist = self.get_L1_distance(velocity)
        lat_acc_cmd = 2.0 * groundspeed ** 2 / L1_dist * np.sin(eta)

        # --- Desired roll angle ---
        g = 9.80665  # m/s²
        roll_cmd = np.arctan2(lat_acc_cmd, g)

        self.output.airspeed = self.default_airspeed
        self.output.roll = roll_cmd
        self.output.altitude = curr_wp.z

        return self.output

    def get_L1_distance(self, velocity: np.ndarray) -> float:
        groundspeed = self.vector_norm(velocity)
        return max(self.L1_ratio * groundspeed, self.min_L1_dist)
