# Autonomy/Controller/fw_controller.py
import numpy as np
from Autonomy.PID import PID_class
from Global.utils import linear_scale
from Global.simdata import UAVState_class, FW_controls, FW_target, controller_flags_class

class FixedWingController:
    def __init__(self, dt, TECS_control: bool = True):
        self.dt = dt
        self.TECS_control = TECS_control
        self.target: FW_target = FW_target()
        self.output: FW_controls = FW_controls()
        self._init_pids()
        self.g = 9.81

        # tecs parameters
        self.kp_et = 1.0
        self.kd_et = 0.2
        self.kp_eb = 0.5
        self.kd_eb = 0.1
        self.prev_e_total = 0.0
        self.prev_e_balance = 0.0

    def _init_pids(self):
        self.pids = {
            "airspeed": PID_class(),
            "altitude": PID_class(),
            "roll": PID_class(),
            "roll_rate": PID_class(),
            "pitch": PID_class(),
            "pitch_rate": PID_class(),
            "heading": PID_class(),
        }

        # airspeed control
        self.pids["airspeed"].update_gains(kp=10, ki=0.5, kd=0.5)
        self.pids["airspeed"].set_output_limits(lower=-40.0, upper=40.0)
        self.pids["airspeed"].set_integral_limits(lower=-4, upper=4)

        # altitude control
        self.pids["altitude"].update_gains(kp=0.1 / (15), ki=0, kd=0)
        self.pids["altitude"].set_output_limits(lower=(-15), upper=(15))

        self.pids["pitch"].update_gains(kp=1, kd=0, ki=0)
        self.pids["pitch"].set_output_limits(lower=(-15.0), upper=(15.0))

        self.pids["pitch_rate"].update_gains(kp=2.0, kd=0.1, ki=0.1)
        self.pids["pitch_rate"].set_output_limits(lower=(-30.0), upper=(30.0))
        self.pids["pitch_rate"].set_integral_limits(lower=(-3.0), upper=(3.0))

        # roll control
        self.pids["heading"].update_gains(kp=0.3, kd=0.05, ki=0.0)
        self.pids["heading"].set_output_limits(lower=(-30.0), upper=(30.0))

        self.pids["roll"].update_gains(kp=1, kd=0, ki=0.0)
        self.pids["roll"].set_output_limits(lower=(-30.0), upper=(30.0))

        self.pids["roll_rate"].update_gains(kp=1, kd=0.05, ki=0.1)
        self.pids["roll_rate"].set_output_limits(lower=(-30.0), upper=(30.0))
        self.pids["roll_rate"].set_integral_limits(lower=(-10.0), upper=(10.0))

    def tecs(self, h, h_des, V, V_des, dt):
        e_total = (h - h_des) + (V**2 - V_des**2) / (2 * self.g)
        e_balance = (h - h_des) - (V**2 - V_des**2) / (2 * self.g)

        e_total_dot = (e_total - self.prev_e_total) / dt
        e_balance_dot = (e_balance - self.prev_e_balance) / dt

        throttle_cmd = self.kp_et * e_total + self.kd_et * e_total_dot
        pitch_cmd = self.kp_eb * e_balance + self.kd_eb * e_balance_dot

        self.prev_e_total = e_total
        self.prev_e_balance = e_balance

        return np.clip(throttle_cmd, 0, 100), np.clip(pitch_cmd, -30, 30)

    def run(self, current_state: UAVState_class, target: FW_target, controller_flags:controller_flags_class)-> FW_controls:
        # extract the target data
        self.target = target

        # Step 2 - Roll controls loop
        roll_cmd = self.pids["roll"].run_pid(
            self.target.roll, current_state.phi, self.dt
        )
        roll_rate_cmd = self.pids["roll_rate"].run_pid(
            roll_cmd, current_state.phi_rate, self.dt
        )
        self.output.aileron = roll_rate_cmd

        # step 3 - Rudder for coordinated turn (simplified)
        self.output.rudder = -0.7 * self.output.aileron

        # step 4 - Altitude and airspeed control by TECS or traditional method by configs
        if self.TECS_control:
            self.output.throttle, self.output.elevator = self.tecs(
                h=current_state.z,
                h_des=self.target.altitude,
                V=current_state.x_vel,
                V_des=self.target.airspeed,
                dt=self.dt,
            )

        else:
            # --- Vertical Control: Altitude → Pitch → Pitch Rate ---
            pitch_cmd = self.pids["altitude"].run_pid(
                self.target.altitude, current_state.z, self.dt
            )
            pitch_rate_cmd = self.pids["pitch"].run_pid(
                pitch_cmd, current_state.theta, self.dt
            )
            self.output.elevator = -self.pids["pitch_rate"].run_pid(
                pitch_rate_cmd, current_state.theta_rate, self.dt
            )

            # --- Throttle Control: Airspeed loop ---
            self.output.throttle = self.pids["airspeed"].run_pid(
                self.target.airspeed, current_state.x_vel, self.dt
            )

        return self.output
