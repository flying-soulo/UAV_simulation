import numpy as np
from Autonomy.PID import PID_class
from Global.utils import linear_scale
from Global.simdata import FWTarget, FWControlOutputs, ControllerFlags, UAVState
import Global.configs as configs


class FixedWingController:
    def __init__(self, dt, TECS_control: bool = False):
        self.dt = dt
        self.TECS_control = TECS_control
        self.target: FWTarget = FWTarget()
        self.output: FWControlOutputs = FWControlOutputs()
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

        # Airspeed control → affects throttle
        self.pids["airspeed"].update_gains(kp=5.0, ki=0.3, kd=0.2)
        self.pids["airspeed"].set_output_limits(lower=0.0, upper=100.0)  # throttle: 0–100%
        self.pids["airspeed"].set_integral_limits(lower=-10.0, upper=10.0)

        # Altitude control → targets pitch angle
        self.pids["altitude"].update_gains(kp=0.05, ki=0.0, kd=0.0)
        self.pids["altitude"].set_output_limits(lower=-15.0, upper=15.0)

        # Pitch control → targets pitch rate
        self.pids["pitch"].update_gains(kp=1.5, ki=0.0, kd=0.0)
        self.pids["pitch"].set_output_limits(lower=-20.0, upper=20.0)

        # Pitch rate control → drives elevator
        self.pids["pitch_rate"].update_gains(kp=3.0, ki=0.2, kd=0.1)
        self.pids["pitch_rate"].set_output_limits(lower=-100.0, upper=100.0)
        self.pids["pitch_rate"].set_integral_limits(lower=-20.0, upper=20.0)

        # Heading control → outputs desired roll angle
        self.pids["heading"].update_gains(kp=0.6, ki=0.05, kd=0.1)
        self.pids["heading"].set_output_limits(lower=-30.0, upper=30.0)

        # Roll control → targets roll rate
        self.pids["roll"].update_gains(kp=1.5, ki=0.0, kd=0.1)
        self.pids["roll"].set_output_limits(lower=-30.0, upper=30.0)

        # Roll rate control → drives aileron
        self.pids["roll_rate"].update_gains(kp=3.0, ki=0.2, kd=0.1)
        self.pids["roll_rate"].set_output_limits(lower=-100.0, upper=100.0)
        self.pids["roll_rate"].set_integral_limits(lower=-20.0, upper=20.0)


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

    def run(
        self,
        current_state: UAVState,
        target: FWTarget,
        controller_flags: ControllerFlags,
    ) -> FWControlOutputs:
        # extract the target data
        self.target = target

        # Step 2 - Roll controls loop
        roll_cmd = self.pids["roll"].run_pid(
            self.target.roll, current_state.phi, self.dt
        )
        self.output.aileron = self.pids["roll_rate"].run_pid(
            roll_cmd, current_state.phi_rate, self.dt
        )

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

        # Scale aileron output
        self.output.aileron = linear_scale(
            self.output.aileron,
            self.pids["roll_rate"].output_limits[0],
            self.pids["roll_rate"].output_limits[1],
            configs.stick_input_min,
            configs.stick_input_max,
        )

        # Scale elevator output
        self.output.elevator = linear_scale(
            self.output.elevator,
            self.pids["pitch_rate"].output_limits[0],
            self.pids["pitch_rate"].output_limits[1],
            configs.stick_input_min,
            configs.stick_input_max,
        )

        # Scale rudder output
        self.output.rudder = linear_scale(
            self.output.rudder,
            self.pids["roll_rate"].output_limits[
                0
            ],  # Assuming rudder uses roll_rate limits
            self.pids["roll_rate"].output_limits[1],
            configs.stick_input_min,
            configs.stick_input_max,
        )

        # Scale throttle output
        self.output.throttle = linear_scale(
            self.output.throttle,
            self.pids["airspeed"].output_limits[0],
            self.pids["airspeed"].output_limits[1],
            configs.stick_input_min,
            configs.stick_input_max,
        )

        return self.output
