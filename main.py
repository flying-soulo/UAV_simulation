import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from plant.plant import Simulation_class  
from Autopilot.autopilot import Autopilot_class  
from visualize.visual import UAVVisualizer  
from utils import get_aerosnode_properties, wrap, lpf


def run():
    #time
    Tend = 50.0  # seconds, total simulation time
    freq = 100  # Hz
    dt = 1/freq  # Time step in seconds
    t = np.arange(Tend/dt) # Example simulation time steps

    waypoints = ([1000, 2000, -2000, 20, "reach"],
                [-2000, 2000, -1000, 20, "reach"],
                [2000, -1000, -5000, 20, "reach"],
                [1000, 1000, -5000, 20, "reach"],
                [1000, 1000, -3000, 20, "reach"])
    
    # Initialize simulation, autopilot, and visualizer
    vehicle_prop = get_aerosnode_properties()
    simulation = Simulation_class(vehicle_prop)
    autopilot = Autopilot_class(waypoints)
    visualizer = UAVVisualizer()

    #initial conditions for states
    current_state = np.zeros(12)  # Adjust based on actual state size
    update_step = np.zeros(12)
    current_state[2] = -5000 #m in altitude
    current_state[3] = 30 #m/s airpseed

    # Initial conditions for control values
    motor_thrust = np.zeros(5)  # Example for quad motors
    ctrl_srfc_deflection = np.zeros(3)  # Example for control surfaces

    #forces and moments
    forces_moments = np.zeros(6)  # Assuming 6 forces and moments

    simulation_data = {
    "time": [],
    "x": [],  "y": [],  "z": [],
    "u": [],  "v": [],  "w": [],
    "phi": [], "theta": [], "psi": [],
    "p": [],  "q": [],  "r": [],
    "Fx": [], "Fy": [], "Fz": [],
    "l": [],  "m": [],  "n": [],
    "motor0": [], "motor1": [], "motor2": [], "motor3": [], "motor4": [],
    "aileron": [], "elevator": [], "rudder": []
    }

    # Simulation loop
    for i in t:
        # Run one simulation step to get new state
        update_step, forces_moments = simulation.simulate_one_step(current_state, motor_thrust, ctrl_srfc_deflection, dt)

        # Run autopilot to update control inputs
        motor_thrust, ctrl_srfc_deflection = autopilot.run(update_step, dt)

        # Update the visualization with the new state
        visualizer.update_visualization(update_step)

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
        # time.sleep(dt)
    
    df = pd.DataFrame(simulation_data)
    df.to_csv("sim_log/simulation.csv", index=False)

    plot_simulation_states(df)

def plot_simulation_states(states):
    fig, axs = plt.subplots(3, 3, figsize=(12, 15))
    fig.suptitle("Simulation States")

    # Position x, y, z
    axs[0, 0].plot(states["time"], states["x"], label='x (m)', color='r')
    axs[0, 0].set_title("Position x")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("x (m)")
    axs[0, 0].legend()
    axs[0, 0].grid()

    axs[0, 1].plot(states["time"], states["y"], label='y (m)', color='g')
    axs[0, 1].set_title("Position y")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("y (m)")
    axs[0, 1].legend()
    axs[0, 1].grid()

    axs[0, 2].plot(states["time"], states["z"], label='z (m)', color='b')
    axs[0, 2].set_title("Position z")
    axs[0, 2].set_xlabel("Time (s)")
    axs[0, 2].set_ylabel("z (m)")
    axs[0, 2].legend()
    axs[0, 2].grid()

    # velocity u v w
    axs[1, 0].plot(states["time"], states["u"], label='u (m/s)', color='r')
    axs[1, 0].set_title("Position x")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("u (m/s)")
    axs[1, 0].legend()
    axs[1, 0].grid()

    axs[1, 1].plot(states["time"], states["v"], label='v (m/s)', color='g')
    axs[1, 1].set_title("Position y")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("v (m/s)")
    axs[1, 1].legend()
    axs[1, 1].grid()

    axs[1, 2].plot(states["time"], states["w"], label='w (m/s)', color='b')
    axs[1, 2].set_title("Position z")
    axs[1, 2].set_xlabel("Time (s)")
    axs[1, 2].set_ylabel("w (m/s)")
    axs[1, 2].legend()
    axs[1, 2].grid()

    # Angles (phi, theta)
    axs[2, 0].plot(states["time"], states["phi"], label='phi (rad)')
    axs[2, 0].plot(states["time"], states["theta"], label='theta (rad)')
    axs[2, 0].set_title("Angles (phi, theta)")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Angle (rad)")
    axs[2, 0].legend()
    axs[2, 0].grid()

    # Angle (psi)
    axs[2, 1].plot(states["time"], states["psi"], label='psi (rad)')
    axs[2, 1].set_title("Heading angle (psi)")
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].set_ylabel("psi (rad)")
    axs[2, 1].legend()
    axs[2, 1].grid()

    # Angular Rates (p, q, r)
    axs[2, 2].plot(states["time"], states["p"], label='p (rad/s)')
    axs[2, 2].plot(states["time"], states["q"], label='q (rad/s)')
    axs[2, 2].plot(states["time"], states["r"], label='r (rad/s)')
    axs[2, 2].set_title("Angular Rates (p, q, r)")
    axs[2, 2].set_xlabel("Time (s)")
    axs[2, 2].set_ylabel("Angular Rate (rad/s)")
    axs[2, 2].legend()
    axs[2, 2].grid()

    plt.tight_layout()
    plt.savefig("sim_log/simulation.png")
    plt.show()

if __name__ == "__main__":
    run()