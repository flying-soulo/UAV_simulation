import airsim
import time
import numpy as np

class AirSimVisualizer:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.simPause(True)  # Pause physics to override pose
        print("[AirSimVisualizer] Connected and paused physics.")

    def update_pose(self, position, orientation_rpy):
        """
        Update the drone's position and orientation in the simulator.
        :param position: (x, y, z) in meters
        :param orientation_rpy: (roll, pitch, yaw) in radians
        """
        x, y, z = position
        roll, pitch, yaw = orientation_rpy
        pose = airsim.Pose(
            airsim.Vector3r(x, y, z),
            airsim.to_quaternion(pitch, roll, yaw)  # Note the order: P, R, Y
        )
        self.client.simSetVehiclePose(pose, True)

    def shutdown(self):
        self.client.simPause(False)
        print("[AirSimVisualizer] Resumed physics and disconnected.")

# --- Example usage (test loop) ---
if __name__ == "__main__":
    vis = AirSimVisualizer()
    try:
        for i in range(100):
            # Simulate some motion
            position = (i * 0.1, 0, -5)  # move along x
            orientation = (0, 0, i * 0.01)  # slow yaw spin
            vis.update_pose(position, orientation)
            time.sleep(0.05)
    finally:
        vis.shutdown()
