import numpy as np
from Global.utils import wrap
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle


def trim_longitudinal(cruise_speed, vz):
    vehicle_prop = Aerosonde_vehicle.copy()
    current_states = np.zeros(12)  # Placeholder for current states
    current_states[3] = cruise_speed  # Set the cruise speed in the current state
    u = current_states[3]
    v = current_states[4]
    w = current_states[5]
    gamma = np.atan(-vz / (u**2 + v**2) ** 0.5)  # Placeholder for gamma calculation

    Cm_delta_e = vehicle_prop["Cm_delta_e"]
    Cm_alpha = vehicle_prop["Cm_alpha"]
    CL_delta_e = vehicle_prop["CL_delta_e"]
    CL_alpha = vehicle_prop["CL_alpha"]
    Cm0 = vehicle_prop["Cm0"]
    CL0 = vehicle_prop["CL0"]
    m = vehicle_prop["m"]
    g = 9.8
    S = vehicle_prop["S"]
    rho = vehicle_prop["rho"]

    # Compute airspeed and angles
    V = np.sqrt(
        current_states[3] ** 2 + current_states[4] ** 2 + current_states[5] ** 2
    )  # Airspeed magnitude
    alpha = wrap(
        np.arctan2(w, u), -np.pi, np.pi
    )  # Angle of attack wrapped within [-π, π]
    beta = wrap(
        np.arcsin(np.clip(v / V, -1, 1)), -np.pi / 2, np.pi / 2
    )  # Sideslip angle wrapped within [-π/2, π/2]
    q_dyn = 0.5 * rho * V**2  # Dynamic pressure

    B = ([Cm_delta_e, Cm_alpha], [CL_delta_e, CL_alpha])
    C = [-Cm0, ((m * g * np.cos(gamma)) / (q_dyn * S)) - CL0]

    A = np.linalg.inv(B).dot(C)
    alpha = A[1]
    elevator_deflection = A[0]  # Extract elevator deflection from A

    CD0 = vehicle_prop["CD0"]
    CD_alpha = vehicle_prop["CD_alpha"]
    CD_delta_e = vehicle_prop["CD_delta_e"]
    thrust = q_dyn * S * (
        CD0 + CD_alpha * np.abs(alpha) + CD_delta_e * np.abs(elevator_deflection)
    ) + m * 9.8 * np.sin(
        gamma
    )  # Thrust calculation
    return thrust, elevator_deflection
