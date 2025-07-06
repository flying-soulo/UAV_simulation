# simdata.py
import numpy as np
from dataclasses import dataclass, field

########################################################################
################################### GCS ################################
########################################################################


@dataclass
class Waypoint_class:
    """
    waypoint structure with x y z target locations, heading, action, mode, notes(extra)
    """

    x: float = 0
    y: float = 0
    z: float = 0

    heading: float = 0

    action: str = ""
    mode: str = ""
    next: int = 1


@dataclass
class Waypoint_data_class:
    """
    Contains all the waypoints of the simulation
    """

    home: Waypoint_class = field(default_factory=Waypoint_class)
    waypoints: list[Waypoint_class] = field(default_factory=lambda: [Waypoint_class()])


@dataclass
class Radio_data_class:
    channel1: float = 0
    channel2: float = 0
    channel3: float = 0
    channel4: float = 0
    mode_switch: str = "QD_MANUAL"


@dataclass
class GCSData_class:
    """
    GCS data structure which has all the data from the GCS
    """

    sim_command: str = ""
    mode: str = ""
    command: str = ""

    waypoint_data: Waypoint_data_class = field(
        default_factory=lambda: Waypoint_data_class()
    )
    current_waypoint: int = 0

    radio: Radio_data_class = field(default_factory=Radio_data_class)


########################################################################
############################### Autopilot ##############################
########################################################################
@dataclass
class FW_target:
    roll: float = 0
    altitude: float = 0
    airspeed: float = 0


@dataclass
class Quad_target:
    x: float = 0
    y: float = 0
    altitude: float = 0
    heading: float = 0


@dataclass
class Target_data_struct:
    Quad: Quad_target = field(default_factory=Quad_target)
    FW: FW_target = field(default_factory=FW_target)


@dataclass
class Mission_track_data:
    curr_wp: Waypoint_class = field(default_factory=Waypoint_class)
    prev_wp: Waypoint_class = field(default_factory=Waypoint_class)


@dataclass
class Quad_controls:
    throttle: float = 0
    roll: float = 0
    pitch: float = 0
    yaw: float = 0


@dataclass
class FW_controls:
    throttle: float = 0
    aileron: float = 0
    elevator: float = 0
    rudder: float = 0


@dataclass
class Controls_class:
    """
    contains the controller ouputs from Autopilot
    """

    FW: FW_controls = field(default_factory=FW_controls)
    Quad: Quad_controls = field(default_factory=Quad_controls)


@dataclass
class controller_flags_class:
    """
    Controller flags
    """

    angle_controller: bool = False
    angle_rate_controller: bool = False
    throttle: bool = False
    mode: str = ""


@dataclass
class Controller_reset_flags:
    reset_pos_int_x: bool = False
    reset_pos_int_y: bool = False
    reset_vel_int_x: bool = False
    reset_vel_int_y: bool = False
    reset_pos_int_z: bool = False
    reset_vel_int_z: bool = False

    reset_angle_int_roll: bool = False
    reset_angle_int_pitch: bool = False
    reset_angle_rate_int_roll: bool = False
    reset_angle_rate_int_pitch: bool = False

    reset_angle_int_yaw: bool = False
    reset_angle_rate_int_yaw: bool = False


########################################################################
############################## Simulation ##############################
########################################################################
@dataclass
class Quad_actuator:
    Motor1: float = 0
    Motor2: float = 0
    Motor3: float = 0
    Motor4: float = 0


@dataclass
class FW_actuator:
    throttle: float = 0
    aileron: float = 0
    elevator: float = 0
    rudder: float = 0


@dataclass
class Actuator_class:
    """
    Data class for the values of the UAV control values
    """

    # QUad controls
    Quad: Quad_actuator = field(default_factory=Quad_actuator)

    # FW Controls
    FW: FW_actuator = field(default_factory=FW_actuator)


@dataclass
class UAVState_class:
    """
    State variables of the UAV
    """

    systemArmed = bool = False
    flight_mode = str = "idle"

    x: float = 0
    y: float = 0
    z: float = 0

    x_vel: float = 0
    y_vel: float = 0
    z_vel: float = 0

    phi: float = 0
    theta: float = 0
    psi: float = 0

    phi_rate: float = 0
    theta_rate: float = 0
    psi_rate: float = 0

    airspeed: float = 0


@dataclass
class UAVForce_class:
    """
    Force values realted to the simulation
    """

    # Aerodynamics forces
    lift: float = 0
    drag: float = 0

    # Body frame forces
    Fx: float = 0
    Fy: float = 0
    Fz: float = 0

    # Body moments
    l_moment: float = 0
    m_moment: float = 0
    n_moment: float = 0
