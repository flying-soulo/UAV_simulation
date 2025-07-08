import numpy as np
from Global.simdata import FW_target, Mission_track_data, UAVState_class

class FW_guidance:
    def __init__(self, min_L1_dist:float=300.0, L1_ratio:float=1.5, default_airspeed:float=30.0):
        self.min_L1_dist = min_L1_dist
        self.L1_ratio = L1_ratio
        self.default_airspeed = default_airspeed

    def get_L1_distance(self, v):
        gs = float(np.linalg.norm(v))
        return max(self.L1_ratio * gs, self.min_L1_dist)

    def run(self, state: UAVState_class, mission_track: Mission_track_data) -> FW_target:
        prev = mission_track.prev_wp
        curr = mission_track.curr_wp
        pos = np.array([state.x, state.y])
        vel = np.array([state.x_vel, state.y_vel])

        path_vec = np.array([curr.x - prev.x, curr.y - prev.y])
        path_unit = path_vec / (np.linalg.norm(path_vec) + 1e-6)

        pos_vec = pos - np.array([prev.x, prev.y])
        along_track = np.dot(pos_vec, path_unit)
        L1_vec = (along_track + self.get_L1_distance(vel)) * path_unit - pos_vec

        eta = np.arctan2(L1_vec[1], L1_vec[0]) - np.arctan2(vel[1], vel[0])
        eta = np.arctan2(np.sin(eta), np.cos(eta))
        gs = np.linalg.norm(vel)
        a_lat = 2.0 * gs ** 2 / self.get_L1_distance(vel) * np.sin(eta)

        g = 9.81
        roll = np.arctan2(a_lat, g)

        return FW_target(roll=roll, altitude=curr.z, airspeed=self.default_airspeed)
