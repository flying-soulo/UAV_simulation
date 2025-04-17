import numpy as np

from Global.utils import wrap, rotation_matrix

def forces_moments_calc(vehicle_prop, state, motor_thrust, ctrl_srfc_deflection):
    '''
    This function calculates the forces and moments acting on the vehicle in the body frame.
    '''
    # angles of the vehicle 
    phi = state[6]  # Roll angle
    theta = state[7]  # Pitch angle
    psi = state[8]  # Yaw angle
    
    # Body frame velocities
    u = state[3]  # Velocity in body x-axis
    v = state[4]  # Velocity in body y-axis
    w = state[5]  # Velocity in body z-axis
    
    # Angular rates
    p = state[9]  # Roll rate
    q = state[10]  # Pitch rate
    r = state[11]  # Yaw rate

    # Control surface deflections for Fixed wing
    aileron_deflection = ctrl_srfc_deflection[0]  # Aileron control surface
    elevator_deflection = ctrl_srfc_deflection[1]  # Elevator control surface
    rudder_deflection = ctrl_srfc_deflection[2]  # Rudder control surface
    thrust_FW = motor_thrust[4]  # Throttle control surface
    
    # Thrust values for quad plane in quad copter mode
    thrust_LF = motor_thrust[0]  # Throttle of motor0 for quad plane in quad copter mode
    thrust_RF = motor_thrust[1]  # Throttle of motor1 for quad plane in quad copter mode
    thrust_RB = motor_thrust[2]  # Throttle of motor2 for quad plane in quad copter mode
    thrust_LB = motor_thrust[3]  # Throttle of motor3 for quad plane in quad copter mode
    
    # vehicle properties
    m = vehicle_prop['m']  # Mass in kg
    Jx = vehicle_prop['Jx']  # Moment of inertia around x-axis in kg-m^2
    Jy = vehicle_prop['Jy']  # Moment of inertia around y-axis in kg-m^2
    Jz = vehicle_prop['Jz']  # Moment of inertia around z-axis in kg-m^2
    Jxz = vehicle_prop['Jxz']  # Product of inertia in kg-m^2
    S = vehicle_prop['S']  # Wing area in m^2
    b = vehicle_prop['b']  # Wing span in m
    c = vehicle_prop['c']  # Mean aerodynamic chord in m
    rho = vehicle_prop['rho']  # Air density in kg/m^3
    e = vehicle_prop['e']  # Oswald efficiency factor

    # Logntudinal Aerodynamic coefficients
    CL0 = vehicle_prop['CL0']
    CD0 = vehicle_prop['CD0']
    Cm0 = vehicle_prop['Cm0']
    CL_alpha = vehicle_prop['CL_alpha']
    CD_alpha = vehicle_prop['CD_alpha']
    Cm_alpha = vehicle_prop['Cm_alpha']
    CLq = vehicle_prop['CLq']
    CDq = vehicle_prop['CDq']
    Cmq = vehicle_prop['Cmq']
    CL_delta_e = vehicle_prop['CL_delta_e']
    CD_delta_e = vehicle_prop['CD_delta_e']
    Cm_delta_e = vehicle_prop['Cm_delta_e']
    M = vehicle_prop['M']
    alpha0 = vehicle_prop['alpha0']
    CDp = vehicle_prop['CDp']

    # Lateral Aerodynamic coefficients
    CY0 = vehicle_prop['CY0']
    Cl0 = vehicle_prop['Cl0']
    Cn0 = vehicle_prop['Cn0']
    CY_beta = vehicle_prop['CY_beta']
    Cl_beta = vehicle_prop['Cl_beta']
    Cn_beta = vehicle_prop['Cn_beta']
    CYp = vehicle_prop['CYp']
    Clp = vehicle_prop['Clp']
    Cnp = vehicle_prop['Cnp']
    CYr = vehicle_prop['CYr']
    Clr = vehicle_prop['Clr']
    Cnr = vehicle_prop['Cnr']
    CY_delta_a = vehicle_prop['CY_delta_a']
    Cl_delta_a = vehicle_prop['Cl_delta_a']
    Cn_delta_a = vehicle_prop['Cn_delta_a']
    CY_delta_r = vehicle_prop['CY_delta_r']
    Cl_delta_r = vehicle_prop['Cl_delta_r']
    Cn_delta_r = vehicle_prop['Cn_delta_r']

    # Quadcopter thrust coefficients
    arm = vehicle_prop['arm_length'] / 2  # Arm length in m
    c_tau = vehicle_prop['toruq_motor']  # Torque of the motor in N-m
    
    # Compute airspeed and angles
    V = np.sqrt(u**2 + v**2 + w**2)  # Airspeed magnitude
    alpha = wrap(np.arctan2(w, u), -np.pi, np.pi)  # Angle of attack wrapped within [-π, π]
    beta = wrap(np.arcsin(np.clip(v / V, -1, 1)), -np.pi / 2, np.pi / 2)  # Sideslip angle wrapped within [-π/2, π/2]
    q_dyn = 0.5 * rho * V**2  # Dynamic pressure

    # Rotation matrices
    R_ned_to_body = rotation_matrix(phi, theta, psi)  # NED to body frame
    R_stb_to_body = rotation_matrix(0, -alpha, 0)     # Stability to body frame

    # Gravity in body frame 
    gravity_body = R_ned_to_body @ np.array([0, 0, m * 9.81])

    # Aerodynamic forces in body frame
    CL = (
        CL0 
        + CL_alpha * alpha  
        + CLq * q * c / (2 * V) 
        + CL_delta_e * elevator_deflection
    )  # Lift coefficient

    CY = (
        CY0 
        + CY_beta * beta 
        + CYp * p 
        + CYr * r 
        + CY_delta_a * aileron_deflection 
        + CY_delta_r * rudder_deflection
    )  # Sideslip force coefficient

    CD = (
        CD0 
        + CD_alpha * np.abs(alpha) 
        + CD_delta_e * np.abs(elevator_deflection) 
        + CDq * np.abs(q) * c / (2 * V)
    )  # Drag coefficient

    # Total aerodynamic forces in body frame
    lift = q_dyn * S * CL  # Lift force
    drag = q_dyn * S * CD  # Drag force
    F_aero_body = R_stb_to_body @ np.array([-drag, 0, -lift])  # Correct matrix multiplication
    side_force = q_dyn * S * CY # Side force with safeguard
    F_aero_body[1] += side_force  # Add side force to the y-component

    # Thrust forces in body frame
    thrust_body = np.array([
        thrust_FW,  
        0,          
        -thrust_RB - thrust_LB - thrust_RF - thrust_LF
    ])

    # Total forces in body frame
    Fx, Fy, Fz = F_aero_body + gravity_body + thrust_body
    
    # Aerodynamic coefficients for moments
    Cl = (
        Cl0 
        + Cl_beta * beta 
        + Clp * p * b / (2 * V) 
        + Clr * r * b / (2 * V) 
        + Cl_delta_a * aileron_deflection 
        + Cl_delta_r * rudder_deflection
    )  # Roll moment coefficient

    Cm = (
        Cm0 
        + Cm_alpha * alpha 
        + Cmq * q * c / (2 * V) 
        + Cm_delta_e * elevator_deflection
    )  # Pitch moment coefficient

    Cn = (
        Cn0 
        + Cn_beta * beta 
        + (Cnp * p + Cnr * r) * b / (2 * V) 
        + Cn_delta_a * aileron_deflection 
        + Cn_delta_r * rudder_deflection
    )  # Yaw moment coefficient

    # Aerodynamic moments in body frame
    l_aero = q_dyn * S * b * Cl  # Roll moment due to aerodynamics
    m_aero = q_dyn * S * c * Cm  # Pitch moment due to aerodynamics
    n_aero = q_dyn * S * b * Cn  # Yaw moment
    
    # Thrust moments in body frame (quadcopter contribution) LF is clockwise
    l_thrust = arm * (thrust_LF + thrust_LB - thrust_RF - thrust_RB)  # Roll moment
    m_thrust = arm * (thrust_RF + thrust_LF - thrust_RB - thrust_LB)  # Pitch moment
    n_thrust = c_tau * (thrust_LB + thrust_RF - thrust_LF - thrust_RB)  # Yaw moment

    # Total moments in body frame
    l = l_aero + l_thrust
    m = m_aero + m_thrust
    n = n_aero + n_thrust

    # Return forces and moments in body frame
    return Fx, Fy, Fz, l, m, n
