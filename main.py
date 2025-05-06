import time
import numpy as np
import pandas as pd
from Autonomy.Autopilot import UAVAutopilot
from AeroVehicle.Vehicle_Sim import UAVSimulation
from GUI.interface import UAVinterface
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle
from vpython import rate


class UAVSimulator:
    def __init__(self):
        # Simulation parameters
        self.freq = 100  # Hz
        self.dt = 1 / self.freq

        # Waypoints: (x, y, z, speed, mode)
        self.waypoints = (
            [+5000, +5000, -1000, 30, "reach"],
            [+5000, -5000, -1000, 30, "reach"],
            [-5000, -5000, -1000, 30, "reach"],
        )

        # Initialize vehicle, simulation, autopilot, and interface
        self.vehicle_prop = Aerosonde_vehicle.copy()
        self.simulation = UAVSimulation(self.vehicle_prop, self.dt)
        self.autopilot = UAVAutopilot(self.waypoints, self.dt)
        self.interface = UAVinterface()

        # State and control initialization
        self.current_state = np.zeros(12)
        self.current_state[2] = -1000  # Initial altitude
        self.current_state[3] = 30  # Initial airspeed
        self.update_step = np.zeros(12)

        self.motor_thrust = np.zeros(5)
        self.ctrl_srfc_deflection = np.zeros(3)

        # Keys for logging
        self.keys_state = [
            "x",
            "y",
            "z",
            "u",
            "v",
            "w",
            "phi",
            "theta",
            "psi",
            "p",
            "q",
            "r",
        ]
        self.keys_force_moment = ["Fx", "Fy", "Fz", "l", "m", "n"]
        self.keys_motors = ["motor0", "motor1", "motor2", "motor3", "motor4"]
        self.keys_ctrl_surfaces = ["aileron", "elevator", "rudder"]

        self.simulation_data = {
            key: []
            for key in ["time"]
            + self.keys_state
            + self.keys_force_moment
            + self.keys_motors
            + self.keys_ctrl_surfaces
        }

    def run_simulation(self):
        runsim = False

        while True:
            user_input_data = self.interface.run()
            command = user_input_data["command"]
            mode = user_input_data["mode"]
            waypoint = user_input_data["waypoint"]
            home = user_input_data["home"]

            match (command):
                case "start":
                    runsim = True

                case "pause":
                    runsim = False

                case "stop":
                    runsim = False
                    break

            if runsim:
                # Autopilot computes actuator commands
                control_input = self.autopilot.run(self.current_state)

                # Simulate one step
                self.update_step, forces_moments = self.simulation.simulate_one_step(
                    self.current_state, control_input
                )

                # Visual update
                self.interface.update_uav_visual(self.update_step)

                # Log data
                self.simulation_data["time"].append(time.time())

                for idx, key in enumerate(self.keys_state):
                    self.simulation_data[key].append(self.current_state[idx])

                for idx, key in enumerate(self.keys_force_moment):
                    self.simulation_data[key].append(forces_moments[idx])

                for idx, key in enumerate(self.keys_motors):
                    self.simulation_data[key].append(self.motor_thrust[idx])

                for idx, key in enumerate(self.keys_ctrl_surfaces):
                    self.simulation_data[key].append(self.ctrl_srfc_deflection[idx])
                # Step update
                self.current_state = self.update_step
            rate(self.freq)

        # Save to CSV
        df = pd.DataFrame(self.simulation_data)
        df.to_csv("logger/simulation.csv", index=False)


def main():
    sim = UAVSimulator()
    sim.run_simulation()
    # rate(sim.freq)


if __name__ == "__main__":
    main()
