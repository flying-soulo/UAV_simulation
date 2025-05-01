# Autonomy/Controller/fw_controller.py
import numpy as np
from Autonomy.PID import PID_class
from Global.Utils import linear_scale

class FixedWingController:
    def __init__(self, dt):
        self.dt = dt
        self._init_pids()

    def _init_pids(self):
        self.pids = {
            'airspeed': PID_class(),
            'altitude': PID_class(),
            'roll': PID_class(),
            'roll_rate': PID_class(),
            'pitch': PID_class(),
            'pitch_rate': PID_class(),
            'heading': PID_class(),
        }

        #aitspeed control
        self.pids['airspeed'].update_gains(kp=1.0, ki=0.1, kd=0.01)
        self.pids["airspeed"].set_output_limits(lower=-10.0, upper=10.0)
        self.pids["airspeed"].set_integral_limits(lower=-0.5, upper=0.5)

        # altitude control
        self.pids['altitude'].update_gains(kp=0.3, ki=0, kd=0)
        self.pids['altitude'].set_output_limits(lower=-10.0, upper=10.0)

        self.pids['pitch'].update_gains(kp=1.5, kd=0, ki=0)
        self.pids['pitch'].set_output_limits(lower=np.deg2rad(-15.0), upper=np.deg2rad(15.0))

        self.pids['pitch_rate'].update_gains(kp=2.0, kd=0.1, ki=0.1)
        self.pids['pitch_rate'].set_output_limits(lower=np.deg2rad(-30.0), upper=np.deg2rad(30.0))
        self.pids['pitch_rate'].set_integral_limits(lower=np.deg2rad(-3.0), upper=np.deg2rad(3.0))

        # roll control
        self.pids['heading'].update_gains(kp=0.3, kd=0.05, ki=0.0)
        self.pids['heading'].set_output_limits(lower=np.deg2rad(-30.0), upper=np.deg2rad(30.0))

        self.pids['roll'].update_gains(kp=1, kd=0, ki=0.0)
        self.pids['roll'].set_output_limits(lower=np.deg2rad(-30.0), upper=np.deg2rad(30.0))

        self.pids['roll_rate'].update_gains(kp=1, kd=0.05, ki=0.1)
        self.pids['roll_rate'].set_output_limits(lower=np.deg2rad(-30.0), upper=np.deg2rad(30.0))
        self.pids['roll_rate'].set_integral_limits(lower=np.deg2rad(-10.0), upper=np.deg2rad(10.0))

    def run(self, current_state, nav_state):
        roll, pitch, yaw = current_state[6:9]
        roll_rate, pitch_rate = current_state[9], current_state[10]
        altitude = current_state[2]
        airspeed = current_state[3]

        target_alt = nav_state["target_pos"][2]
        target_airspeed = nav_state["airspeed"]
        target_heading = nav_state["heading"]

        # Altitude Control
        pitch_cmd = -self.pids['altitude'].run_pid(target_alt, altitude, self.dt)
        pitch_cmd = linear_scale(pitch_cmd, -10, 10, np.deg2rad(-30), np.deg2rad(30))
        pitch_rate_cmd = self.pids['pitch'].run_pid(pitch_cmd, pitch, self.dt)
        elevator = -self.pids['pitch_rate'].run_pid(pitch_rate_cmd, pitch_rate, self.dt)

        # Roll Control
        heading_error = (target_heading - yaw + 180) % 360 - 180
        roll_cmd = self.pids['heading'].run_pid(heading_error, 0, self.dt)
        roll_rate_cmd = self.pids['roll'].run_pid(roll_cmd, roll, self.dt)
        aileron = self.pids['roll_rate'].run_pid(roll_rate_cmd, roll_rate, self.dt)


        # side-slip control
        rudder = -0.7 * aileron

        # Airspeed Control
        throttle = self.pids['airspeed'].run_pid(target_airspeed, airspeed, self.dt)


        return {
            "aileron": aileron,
            "elevator": elevator,
            "rudder": rudder,
            "throttle": throttle
        }
