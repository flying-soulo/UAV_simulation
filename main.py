import time
import numpy as np
import pandas as pd
from Autonomy.Autopilot import UAVAutopilot
from AeroVehicle.Vehicle_Sim import UAVSimulation
from GUI.interface import UAVinterface
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle
from vpython import rate
from Global.simdata import UAVForce_class, UAVState_class, Actuator_class, GCSData_class

class UAVSimulator:
    def __init__(self):
        # Simulation parameters
        self.freq = 100  # Hz
        self.dt = 1 / self.freq

        # Waypoints: (x, y, z, speed, mode)
        self.GCS_data : GCSData_class = GCSData_class()
        self.forces_moments : UAVForce_class = UAVForce_class()
        self.Actuators : Actuator_class = Actuator_class()
        self.current_state : UAVState_class = UAVState_class()
        self.update_step : UAVState_class = UAVState_class()

        # State and control initialization
        self.current_state.z = -1000  # Initial altitude
        self.current_state.x_vel = 30  # Initial airspeed

        # Initialize vehicle, simulation, autopilot, and interface
        self.vehicle_prop = Aerosonde_vehicle.copy()
        self.simulation = UAVSimulation(self.vehicle_prop, self.dt)
        self.autopilot = UAVAutopilot(self.GCS_data, self.dt)
        self.interface = UAVinterface()

    def restart(self):
        # Reset only the backend simulation components
        self.GCS_data = GCSData_class()
        self.forces_moments = UAVForce_class()
        self.Actuators = Actuator_class()
        self.current_state = UAVState_class()
        self.update_step = UAVState_class()

        # Reinitialize state variables
        self.current_state.z = -1000
        self.current_state.x_vel = 30

        # Reset vehicle, autopilot, and simulation logic (not GUI)
        self.vehicle_prop = Aerosonde_vehicle.copy()
        self.simulation = UAVSimulation(self.vehicle_prop, self.dt)
        self.autopilot = UAVAutopilot(self.GCS_data, self.dt)

    def run_simulation(self):
        runsim = False

        while True:
            self.GCS_data = self.interface.run()


            match (self.GCS_data.sim_command):
                case "start":
                    runsim = True
                    self.GCS_data.mode = "Auto"

                case "pause":
                    runsim = False

                case "reset":
                    self.restart()
                    runsim = False
                    continue

                case "stop":
                    break

            if runsim:
                # Autopilot computes actuator commands
                control_input = self.autopilot.run(self.current_state, self.GCS_data)

                # Simulate one step
                self.update_step, self.forces_moments = self.simulation.simulate_one_step(self.current_state, control_input)

                # Visual update
                self.interface.update_uav_visual(self.update_step)

                # Step update
                self.current_state = self.update_step
            rate(self.freq)

        # # Save to CSV
        # df = pd.DataFrame(self.simulation_data)
        # df.to_csv("logger/simulation.csv", index=False)


def main():
    sim = UAVSimulator()
    sim.run_simulation()
    # rate(sim.freq)


if __name__ == "__main__":
    main()
