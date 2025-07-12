from dataclasses import dataclass, field
from typing import List

########################################################################
################################### GCS ################################
########################################################################


@dataclass
class Waypoint:
    x: float = 0
    y: float = 0
    z: float = 0
    heading: float = 0
    action: str = ""
    mode: str = ""
    next: int = 1


@dataclass
class RCInput:
    roll: float = 0
    pitch: float = 0
    throttle: float = 0
    yaw: float = 0
    mode_switch: str = "QD_MANUAL"


@dataclass
class MissionTrack:
    target: Waypoint = field(default_factory=Waypoint)
    previous: Waypoint = field(default_factory=Waypoint)


@dataclass
class MissionPlan:
    home: Waypoint = field(default_factory=Waypoint)
    waypoints: List[Waypoint] = field(default_factory=list)
    current_index: int = 0
    previous_index: int = 0
    track: MissionTrack = field(default_factory=MissionTrack)

    def update_track(self):
        if self.waypoints:
            self.previous_index = int(self.waypoints[self.current_index].next)
            if 0 <= self.current_index < len(self.waypoints):
                self.track.previous = self.waypoints[self.current_index]
            if 0 <= self.previous_index < len(self.waypoints):
                self.track.target = self.waypoints[self.previous_index]


@dataclass
class GCSData:
    sim_command: str = "NONE"  # START, PAUSE, STOP
    mode: str = "NONE"  # AUTO, STABILIZE, etc.
    command: str = "NONE"  # ARM, DISARM, etc.
    rc: RCInput = field(default_factory=RCInput)
    mission: MissionPlan = field(default_factory=MissionPlan)


########################################################################
################################### GCS ################################
########################################################################


@dataclass
class FWTarget:
    roll: float = 0
    airspeed: float = 0
    altitude: float = 0


@dataclass
class QuadTarget:
    x: float = 0
    y: float = 0
    altitude: float = 0
    heading: float = 0


@dataclass
class TargetSetpoints:
    quad: QuadTarget = field(default_factory=QuadTarget)
    fw: FWTarget = field(default_factory=FWTarget)


@dataclass
class FWControlOutputs:
    throttle: float = 0
    aileron: float = 0
    elevator: float = 0
    rudder: float = 0


@dataclass
class QuadControlOutputs:
    throttle: float = 0
    roll: float = 0
    pitch: float = 0
    yaw: float = 0


@dataclass
class ControlOutputs:
    fw: FWControlOutputs = field(default_factory=FWControlOutputs)
    quad: QuadControlOutputs = field(default_factory=QuadControlOutputs)


@dataclass
class ControllerFlags:
    angle_ctrl_enabled: bool = False
    angle_rate_ctrl_enabled: bool = False
    throttle_enabled: bool = False
    current_mode: str = ""


@dataclass
class ControllerResetFlags:
    reset_int: dict = field(
        default_factory=lambda: {
            "pos_x": False,
            "pos_y": False,
            "pos_z": False,
            "vel_x": False,
            "vel_y": False,
            "vel_z": False,
            "angle_roll": False,
            "angle_pitch": False,
            "angle_yaw": False,
            "rate_roll": False,
            "rate_pitch": False,
            "rate_yaw": False,
        }
    )


########################################################################
################################### GCS ################################
########################################################################


@dataclass
class UAVState:
    armed: bool = False
    flight_mode: str = "IDLE"

    x: float = 0
    y: float = 0
    z: float = 0

    x_vel: float = 0
    y_vel: float = 0
    z_vel: float = 0

    phi: float = 0  # roll
    theta: float = 0  # pitch
    psi: float = 0  # yaw

    phi_rate: float = 0
    theta_rate: float = 0
    psi_rate: float = 0

    airspeed: float = 0


@dataclass
class QuadActuators:
    motor1: float = 0
    motor2: float = 0
    motor3: float = 0
    motor4: float = 0


@dataclass
class FWActuators:
    throttle: float = 0
    aileron: float = 0
    elevator: float = 0
    rudder: float = 0


@dataclass
class ActuatorOutputs:
    quad: QuadActuators = field(default_factory=QuadActuators)
    fw: FWActuators = field(default_factory=FWActuators)


@dataclass
class UAVForces:
    # Aero forces
    lift: float = 0
    drag: float = 0

    # Body-frame forces
    fx: float = 0
    fy: float = 0
    fz: float = 0

    # Moments
    l: float = 0
    m: float = 0
    n: float = 0
