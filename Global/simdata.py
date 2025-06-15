# simdata.py
import numpy as np
from dataclasses import dataclass, field
from typing import Dict

########################################################################
################################### GCS ################################
########################################################################


@dataclass
class Waypoint_class:
    """
    waypoint structure with x y z target locations, heading, action, mode, notes(extra)
    """

    x: float = 0
    y: float  = 0
    z: float = 0

    heading: float = 0

    action: str = ""
    mode: str =""
    next: int = 1


@dataclass
class Waypoint_data_class:
    """
    Contains all the waypoints of the simulation
    """

    home: Waypoint_class = field(default_factory=Waypoint_class)
    waypoints: list[Waypoint_class] = field(default_factory = lambda: [Waypoint_class()])
    loop: bool = False


@dataclass
class Radio_data_class:
    radio_channel1: float = 0
    radio_channel2: float = 0
    radio_channel3: float = 0
    radio_channel4: float = 0


@dataclass
class GCSData_class:
    """
    GCS data structure which has all the data from the GCS
    """

    sim_command: str = ""
    mode: str = "idle"
    state: str = "idle"
    command: str = ""

    waypoint_data: Waypoint_data_class = field(default_factory=lambda: Waypoint_data_class())
    update_waypoint: bool = False
    current_waypoint: int = 0

    radio_data: Radio_data_class = field(default_factory=Radio_data_class)


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


########################################################################
############################## Simulation ##############################
########################################################################

@dataclass
class Actuator_class:
    """
    Data class for the values of the UAV control values
    """

    # QUad controls
    Quad_Motor1: float = 0
    Quad_Motor2: float = 0
    Quad_Motor3: float = 0
    Quad_Motor4: float = 0

    # FW Controls
    FW_throttle: float = 0
    FW_aileron: float = 0
    FW_elevator: float = 0
    FW_rudder: float = 0


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

    airspeed: float  = 0


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


