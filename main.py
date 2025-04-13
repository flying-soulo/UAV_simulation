import time
import numpy as np
from plant.plant import Simulation_class  # Replace with actual module
from Autopilot.autopilot import Autopilot_class  # Replace with actual module
from visualize.visual import UAVVisualizer  # Fixed typo in 'visualization'
from utils import get_aerosnode_properties, wrap, lpf
import pandas as pd

def run(Tend):
    # Initialize simulation, autopilot, and visualizer
    vehicle_prop = get_aerosnode_properties()
    simulation = Simulation_class(vehicle_prop)
    waypoints = ([1000, 2000, -2000, 20, "reach"],
                [-2000, 2000, -1000, 20, "reach"],
                [2000, -1000, -5000, 20, "reach"],
                [1000, 1000, -5000, 20, "reach"],
                [1000, 1000, -3000, 20, "reach"])
    autopilot = Autopilot_class(waypoints)
    visualizer = UAVVisualizer()

    # Initial conditions
    current_state = np.zeros(18)  # Adjust based on actual state size
    motor_thrust = np.zeros(5)  # Example for quad motors
    ctrl_srfc_deflection = np.zeros(3)  # Example for control surfaces

    #time
    freq = 100  # Hz
    dt = 1/freq  # Time step in seconds
    t = np.arange(Tend/dt) # Example simulation time steps

    #for initial conditions
    current_state[2] = -5000 #m in altitude
    current_state[3] = 30 #m/s airpseed
    update_step = np.zeros(18)

    #boundary conditions for propulsion and control surfaces
    min_thrust, max_thrust = 0, 200  # Adjust based on engine limits
    min_deflection, max_deflection = np.radians(-20), np.radians(20)  # Degrees for control surfaces


    # Dictionary to store simulation results with variable names as keys
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
        "Fx": [],  # Force in x-direction
        "Fy": [],  # Force in y-direction
        "Fz": [],  # Force in z-direction
        "l": [],   # Moment about x-axis
        "m": [],   # Moment about y-axis
        "n": [],   # Moment about z-axis
    }

    for i in t:  # or use a while loop for continuous simulation
        # Run one simulation step to get new state
        update_step = simulation.simulate_one_step(current_state, motor_thrust, ctrl_srfc_deflection, dt)

        # Run autopilot to update control inputs
        motor_thrust, ctrl_srfc_deflection = autopilot.run(update_step, dt)

        # # Apply the low-pass filter to thrust and deflection values
        # motor_thrust = lpf(motor_thrust, simulation_data["Fx"][-len(motor_thrust):], alpha=0.1)
        # ctrl_srfc_deflection = lpf(ctrl_srfc_deflection, simulation_data["Fy"][-len(ctrl_srfc_deflection):], alpha=0.1)

        # Wrap the filtered values within their respective bounds
        motor_thrust = [wrap(value, min_thrust, max_thrust) for value in motor_thrust]
        ctrl_srfc_deflection = [wrap(value, min_deflection, max_deflection) for value in ctrl_srfc_deflection]

        # Update the visualization with the new state
        visualizer.update_visualization(update_step)

        # Prepare the next simulation state
        current_state = update_step

        # Log data into a CSV file
        simulation_data["time"].append(i * dt)  # Add time step scaled by dt
        # Append data to a log or dictionary as needed
        for idx, key in enumerate(["x", "y", "z", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r"]):
            simulation_data[key].append(update_step[idx])

        forces_moments = update_step[12:18]  # Assuming forces and moments are in indices 12 to 17
        for idx, key in enumerate(["Fx", "Fy", "Fz", "l", "m", "n"]):
            simulation_data[key].append(forces_moments[idx])
        
    df = pd.DataFrame(simulation_data)
    df.to_csv("simulation.csv", index=False)

    # Maintain real-time pace
    time.sleep(dt)


if __name__ == "__main__":
    run(10)