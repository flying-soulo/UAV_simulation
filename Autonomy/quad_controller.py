# Autonomy/Controller/quad_controller.py
import numpy as np
from Autonomy.PID import PID_class

class QuadController:
    def __init__(self, dt):
        self.dt = dt
        self._init_pids()

    def _init_pids(self):
        self.pids = {
            f"{a}_pos": PID_class() for a in ['x', 'y', 'z']
        }
        self.pids.update({
            f"{a}_vel": PID_class() for a in ['x', 'y', 'z']
        })
        for axis in ['phi', 'theta', 'psi']:
            self.pids[axis] = PID_class()
            self.pids[f"{axis}_rate"] = PID_class()

    def _run_nested_pid(self, outer_pid, inner_pid, target_outer, current_outer, current_inner):
        mid = outer_pid.run_pid(target_outer, current_outer, self.dt)
        return inner_pid.run_pid(mid, current_inner, self.dt)

    def _run_axis(self, axis, target_pos, current_pos, current_vel):
        return self._run_nested_pid(self.pids[f"{axis}_pos"], self.pids[f"{axis}_vel"], target_pos, current_pos, current_vel)

    def run(self, current_state, nav_state):
        x, y, z = current_state[0:3]
        vx, vy, vz = current_state[3:6]
        phi, theta, psi = current_state[6:9]
        p, q, r = current_state[9:12]
        tx, ty, tz = nav_state["target_pos"]
        th = nav_state["target_heading"]

        t_phi = self._run_axis('x', tx, x, vx)
        t_theta = self._run_axis('y', ty, y, vy)
        base = self._run_axis('z', tz, z, vz)

        roll = self._run_nested_pid(self.pids['phi'], self.pids['phi_rate'], t_phi, phi, p)
        pitch = self._run_nested_pid(self.pids['theta'], self.pids['theta_rate'], t_theta, theta, q)
        yaw = self._run_nested_pid(self.pids['psi'], self.pids['psi_rate'], th, psi, r)

        return {"base":base,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
                }
