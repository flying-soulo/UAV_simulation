import numpy as np
from scipy.spatial.transform import Rotation as R
from vpython import vector, color, box, cone, compound

_R_NED2EUS = np.array([
    [0, 1, 0],     # NED Y → EUS X
    [0, 0, -1],    # NED -Z → EUS Y
    [-1, 0, 0],     # NED X → EUS Z
])

def ned_to_eus(position_ned: np.ndarray, euler_ned_deg: np.ndarray):
    """
    Convert NED position + Euler angles (ZYX: yaw, pitch, roll) to EUS position and Rotation.
    """
    # Position transform
    pos_ned = np.asarray(position_ned)
    pos_eus = _R_NED2EUS @ pos_ned

    # Euler angles in NED: ZYX (yaw, pitch, roll)
    rot_ned = R.from_euler("zyx", euler_ned_deg, degrees=True)


    # Transform body axes
    x_b_ned = rot_ned.apply([1, 0, 0])  # Forward
    y_b_ned = rot_ned.apply([0, 1, 0])  # Right
    z_b_ned = rot_ned.apply([0, 0, 1])  # Down

    x_b_eus = _R_NED2EUS @ x_b_ned
    y_b_eus = _R_NED2EUS @ y_b_ned
    z_b_eus = _R_NED2EUS @ z_b_ned

    # Re-orthonormalize
    x_b_eus = x_b_eus / np.linalg.norm(x_b_eus)
    z_b_eus = np.cross(x_b_eus, y_b_eus)
    z_b_eus = z_b_eus / np.linalg.norm(z_b_eus)
    y_b_eus = np.cross(z_b_eus, x_b_eus)

    # Compose rotation matrix
    rot_eus_matrix = np.column_stack([x_b_eus, y_b_eus, z_b_eus])
    rot_eus = R.from_matrix(rot_eus_matrix)
    return pos_eus, rot_eus
