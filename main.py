import time
import numpy as np
import pandas as pd
from Autonomy.Autopilot import UAVAutopilot
from GUI.visual import UAVSystem
from AeroVehicle.Vehicle_Sim import UAVSimulation
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle
from vpython import rate

# Time setup
freq = 100  # Hz
dt = 1 / freq

# Waypoints: (x, y, z, speed, mode)
waypoints = (
    [+5000, +5000, -1000, 30, "reach"],
    [+5000, -5000, -1000, 30, "reach"],
    [-5000, -5000, -1000, 30, "reach"],
    [-5000, +5000, -1000, 35, "reach"],
    [+5000, +5000, -1000, 35, "reach"],
)

# Vehicle properties and modules
vehicle_prop = Aerosonde_vehicle.copy()
simulation = UAVSimulation(vehicle_prop, dt)
autopilot = UAVAutopilot(waypoints, dt)
visualizer = UAVSystem()
# visualizer.run()  # Uncomment if needed

# Initial state: [x, y, z, u, v, w, phi, theta, psi, p, q, r]
current_state = np.zeros(12)
current_state[2] = -1000  # Initial altitude (z)
current_state[3] = 23  # Initial airspeed (u)
update_step = np.zeros(12)

# Initial actuator states
motor_thrust = np.zeros(5)
ctrl_srfc_deflection = np.zeros(3)

# Constraints
thrust_max = 110 * 0.3
thrust_min = 0
deflection_max = np.deg2rad(30)
deflection_min = np.deg2rad(-30)

# Preallocate log dictionary
keys_state = ["x", "y", "z", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r"]
keys_force_moment = ["Fx", "Fy", "Fz", "l", "m", "n"]
keys_motors = ["motor0", "motor1", "motor2", "motor3", "motor4"]
keys_ctrl_surfaces = ["aileron", "elevator", "rudder"]

simulation_data = {
    key: []
    for key in ["time"]
    + keys_state
    + keys_force_moment
    + keys_motors
    + keys_ctrl_surfaces
}

def run():
    # Simulation loop

    while runsim is True:
        # Simulation loop
        # Autopilot computes actuator commands
        motor_thrust, ctrl_srfc_deflection = autopilot.run(update_step)

        # Enforce actuator limits
        motor_thrust = np.clip(motor_thrust, thrust_min, thrust_max)
        ctrl_srfc_deflection = np.clip(
            ctrl_srfc_deflection, deflection_min, deflection_max
        )

        # Simulate one step
        update_step, forces_moments = simulation.simulate_one_step(
            current_state, motor_thrust, ctrl_srfc_deflection
        )

        visualizer.run(update_step)

        # Log data
        simulation_data["time"].append(time.time())

        for idx, key in enumerate(keys_state):
            simulation_data[key].append(current_state[idx])

        for idx, key in enumerate(keys_force_moment):
            simulation_data[key].append(forces_moments[idx])

        for idx, key in enumerate(keys_motors):
            simulation_data[key].append(motor_thrust[idx])

        for idx, key in enumerate(keys_ctrl_surfaces):
            simulation_data[key].append(ctrl_srfc_deflection[idx])

        # Update current state
        current_state = update_step

        # Real-time pacing
        time.sleep(dt)

    # Save to CSV
    df = pd.DataFrame(simulation_data)
    df.to_csv("logger/simulation.csv", index=False)
    # plot(df)  # Uncomment if plotting is implemented


if __name__ == "__main__":
    while True:
        run(100)
        rate(100)
