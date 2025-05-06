"""
aircarft.py - generate vppython object for the aircaft and sets pose

"""

from vpython import box, cone, compound, vector, color, cylinder
import numpy as np

class Aircraft:
    def __init__(self, length=30, wingspan=35, tailspan=10, tail_height=5):
        # All parts built in local body frame (X-forward, Y-right, Z-down)
        body = cylinder(pos=vector(-length/2,0,0), axis=vector(length,0,0), radius=1.5, color=color.white, emissive=True)
        nose = cone(pos=vector(-length/2,0,0), axis=vector(-4,0,0), radius=1.5, color=color.white, emissive=True)
        wing = box(pos=vector(0,0,0), size=vector(4, wingspan, 0.5), color=color.blue)
        tail = box(pos=vector(length/2,0, 0), size=vector(3, tailspan, 0.4), color=color.red, emissive=True)
        fin  = box(pos=vector(length/2, 0, 2), size=vector(1.5, 0.4, tail_height), color=color.red, emissive=True)
        self.body = compound([body, nose, wing, tail, fin], emissive=True)

    def set_pose(self, position_eus: np.ndarray, rot_eus):
        """
        position_eus : (3,) array in EUS
        rot_eus      : scipy Rotation object (body → EUS)
        """
        M = rot_eus.as_matrix()

        # VPython convention: axis = direction of +Z, up = +Y
        # So we map body-frame:
        #   X (forward) → -Z
        #   Y (right)   → +Y
        fwd = M @ np.array([1, 0, 0])   # body X (forward)
        up  = M @ np.array([0, 1, 0])   # body Y (right)

        self.body.pos  = vector(*position_eus)
        self.body.axis = -vector(*fwd)  # body X → -Z
        self.body.up   = vector(*up)    # body Y → +Y

