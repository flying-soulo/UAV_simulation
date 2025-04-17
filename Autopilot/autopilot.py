import numpy as np
from Autopilot.Controller import MasterController
from Autopilot.waypoint import WaypointNavigator

class Autopilot_class:
    """
    Autopilot class that controls throttle and control surface PWM outputs.
    """

    def __init__(self, waypoints):
        self.mode = "FW"  # 'FW' for fixed-wing; can be extended to include 'QUAD' or 'TRANSITION'
        self.throttle = np.zeros(5)  # Example: 4 quadrotor motors and 1 fixed-wing motor
        self.ctrl_srfc_pwm = np.zeros(3)  # For aileron, elevator, rudder
        self.master_ctrl = MasterController()
        self.navigator = WaypointNavigator(waypoints)

    def run(self, current_state, dt):
        """
        Compute throttle and control surface commands based on UAV mode.

        Args:
            current_state (np.array): Current state vector (assumed 12 elements).

        Returns:
            tuple: (throttle, ctrl_srfc_pwm)
        """
        # Retrieve target values from the WaypointNavigator
        current_waypoint = self.navigator.get_current_waypoint()
        target_x, target_y, target_z, target_airspeed, type = current_waypoint
        target = np.array([target_x, target_y, target_z])
        current_pos = np.array([current_state[0], current_state[1], current_state[2]])
        target_heading = self.navigator.get_waypoint_heading((target), (current_pos))

        if self.mode == "FW":
            self.throttle[4], self.ctrl_srfc_pwm = self.master_ctrl.run_fixed_wing_mode(current_state, target_airspeed, target_heading, target_z, dt)
        elif self.mode == "QUAD":
            target_x = target_position[0]
            target_y = target_position[1]
            target_z = target_altitude
            self.throttle, self.ctrl_srfc_pwm = self.master_ctrl.run_quad_mode(current_state, target_x, target_y, target_z, target_heading, dt)
        else:
            raise ValueError("Invalid mode. Must be 'FW' or 'QUAD'.")

        return self.throttle, self.ctrl_srfc_pwm

# test
if __name__ == "__main__":
    autopilot = Autopilot_class(mode="FW")

    # Create a dummy state vector with 12 elements (indices must match controller assumptions)
    current_state = np.zeros(12)
    current_state[3] = 20.0  # Current airspeed
    current_state[2] = 100.0  # Current altitude
    current_state[11] = 1.57  # Current heading (psi)
    current_state[6:9] = np.array(
        [0.1, 0.05, 1.57]
    )  # Current attitude (phi, theta, psi)
    current_state[9:12] = np.array([0.01, 0.02, 0.03])  # Current rates (p, q, r)

    target_airspeed = 25.0
    target_altitude = 120.0
    target_heading = 1.8
    target_position = np.array(
        [100, 200, 120]
    )  # Target position (x, y, z) if quad mode is used

    throttle, ctrl_srfc_pwm = autopilot.run(current_state)
    print("Throttle:", throttle)
    print("Control Surface PWM:", ctrl_srfc_pwm)
