import time
import numpy as np
import pandas as pd
from Autonomy.Autopilot import Autopilot_class
from GUI.visual import UAVVisualizer
from AeroVehicle.Vehicle_Sim import Aero_Vehicle_sim
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle


def run(Tend):
    # time
    freq = 100  # Hz
    dt = 1 / freq  # Time step in seconds
    t = np.arange(Tend / dt)  # Example simulation time steps

    waypoints = (
        [+5000, +5000, -1000, 30, "reach"],
        [+5000, -5000, -1000, 20, "reach"],
        [-5000, -5000, -1000, 30, "reach"],
        [-5000, +5000, -1000, 25, "reach"],
        [+5000, +5000, -1000, 25, "reach"],
    )

    vehicle_prop = Aerosonde_vehicle.copy()

    # Initialize simulation, autopilot, and visualizer
    simulation = Aero_Vehicle_sim(vehicle_prop)
    autopilot = Autopilot_class(waypoints)
    visualizer = UAVVisualizer()
    # visualizer.run()

    # initial conditions for states
    current_state = np.zeros(12)  # Adjust based on actual state size
    update_step = np.zeros(12)
    current_state[2] = -1000  # m in altitude
    current_state[3] = 23  # m/s airpseed

    # Initial conditions for control values
    motor_thrust = np.zeros(5)  # Example for quad motors
    thrust_max, thrust_min = 120, 0
    ctrl_srfc_deflection = np.zeros(3)  # Example for control surfaces
    delfection_max, deflection_min = -np.deg2rad(30), np.deg2rad(30)

    # forces and moments
    forces_moments = np.zeros(6)  # array for forces and moments

    simulation_data = {
        "time": [],
        "x": [],
        "y": [],
        "z": [],
        "u": [],
        "v": [],
        "w": [],
        "phi": [],
        "theta": [],
        "psi": [],
        "p": [],
        "q": [],
        "r": [],
        "Fx": [],
        "Fy": [],
        "Fz": [],
        "l": [],
        "m": [],
        "n": [],
        "motor0": [],
        "motor1": [],
        "motor2": [],
        "motor3": [],
        "motor4": [],
        "aileron": [],
        "elevator": [],
        "rudder": [],
    }

    # Simulation loop
    for i in t:
        # Run one simulation step to get new state
        update_step, forces_moments = simulation.simulate_one_step(
            current_state, motor_thrust, ctrl_srfc_deflection, dt
        )
        motor_thrust = np.clip(motor_thrust, thrust_min, thrust_max)
        ctrl_srfc_deflection = np.clip(ctrl_srfc_deflection, deflection_min, delfection_max)

        # Run autopilot to update control inputs
        motor_thrust, ctrl_srfc_deflection = autopilot.run(update_step, dt)

        # Update the visualization with the new state
        visualizer.update_from_states(update_step)

        # Log data into a CSV file
        simulation_data["time"].append(i * dt)

        # Append data to the log
        for idx, key in enumerate(["x", "y", "z", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r"]):
            simulation_data[key].append(current_state[idx])

        for idx, key in enumerate(["Fx", "Fy", "Fz", "l", "m", "n"]):
            simulation_data[key].append(forces_moments[idx])

        for idx, key in enumerate(["motor0", "motor1", "motor2", "motor3", "motor4"]):
            simulation_data[key].append(motor_thrust[idx])

        for idx, key in enumerate(["aileron", "elevator", "rudder"]):
            simulation_data[key].append(ctrl_srfc_deflection[idx])

        # Prepare the next simulation state
        current_state = update_step

        # Maintain real-time pace
        time.sleep(dt)

    df = pd.DataFrame(simulation_data)
    df.to_csv("sim_log/simulation.csv", index=False)

    # plot(df)


if __name__ == "__main__":
    run(Tend=100)
