import time
import csv
import pandas as pd
from pathlib import Path
from datetime import datetime

from AeroVehicle.Vehicle_Sim import UAVSimulation
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle
from Autonomy.Autopilot import UAVAutopilot
from GUI.interface import UAVinterface
from Global.simdata import UAVForces, UAVState, ActuatorOutputs, GCSData, Waypoint


class UAVSimulator:
    def __init__(self):
        # Simulation parameters
        self.freq = 100  # Hz
        self.dt = 1 / self.freq

        self.control_input : ActuatorOutputs = ActuatorOutputs()
        self.forces_moments : UAVForces = UAVForces()
        self.current_state : UAVState = UAVState()
        self.update_step : UAVState = UAVState()
        self.GCS_data : GCSData = GCSData()

        # GCS data initialization
        self.GCS_data.mission.home = Waypoint(x=1000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        self.GCS_data.mission.waypoints = [
            Waypoint(x=5000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=1),
            Waypoint(x=5000, y=5000, z=-1000, heading=0, action="reach", mode="Auto", next=2),
            Waypoint(x=-3000, y=3000, z=-1000, heading=0, action="reach", mode="Auto", next=3),
            Waypoint(x=-3000, y=-3000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        ]
        self.GCS_data.mission.current_index = 0
        self.GCS_data.mission.previous_index = 0

        # State and control initialization
        self.current_state.z = -500  # Initial altitude
        self.current_state.x_vel = 22  # Initial airspeed

        # Initialize vehicle, simulation, autopilot, and interface
        self.vehicle_prop = Aerosonde_vehicle.copy()
        self.simulation = UAVSimulation(self.vehicle_prop, self.dt)
        self.autopilot = UAVAutopilot(self.GCS_data, self.dt)
        self.interface = UAVinterface(self.GCS_data)
        self.data_log = []

    def restart(self):
        # Save the log
        df = pd.DataFrame(self.data_log)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"logger/simulation_{timestamp}.csv"
        df.to_csv(filename, index=False)

        # Reset only the backend simulation components
        self.GCS_data = GCSData()

        # GCS data initialization
        self.GCS_data.mission.home = Waypoint(x=1000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        self.GCS_data.mission.waypoints = [
            Waypoint(x=3000, y=1000, z=-1000, heading=0, action="reach", mode="Auto", next=1),
            Waypoint(x=3000, y=3000, z=-1000, heading=0, action="reach", mode="Auto", next=2),
            Waypoint(x=-3000, y=3000, z=-1000, heading=0, action="reach", mode="Auto", next=3),
            Waypoint(x=-3000, y=-3000, z=-1000, heading=0, action="reach", mode="Auto", next=0)
        ]
        self.GCS_data.mission.current_index = 0
        self.GCS_data.mission.previous_index = 0

        self.forces_moments = UAVForces()
        self.Actuators = ActuatorOutputs()
        self.current_state = UAVState()
        self.update_step = UAVState()

        # Reinitialize state variables
        self.current_state.z = -1000
        self.current_state.x_vel = 30

        # Reset vehicle, autopilot, and simulation logic (not GUI)
        self.simulation = UAVSimulation(self.vehicle_prop, self.dt)
        self.autopilot = UAVAutopilot(self.GCS_data, self.dt)

        # reset data log
        self.data_log = []

    def _simulate_step(self):
        self.control_input = self.autopilot.run(self.current_state, self.GCS_data)
        self.update_step, self.forces_moments = self.simulation.simulate_one_step(self.current_state, self.control_input)
        self.interface.update_uav_visual(self.update_step)
        self.current_state = self.update_step

    def _generate_log_entry(self):
        return {

            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],

            "x": self.current_state.x,
            "y": self.current_state.y,
            "z": self.current_state.z,
            "x_vel": self.current_state.x_vel,
            "y_vel": self.current_state.y_vel,
            "z_vel": self.current_state.z_vel,
            "phi": self.current_state.phi,
            "theta": self.current_state.theta,
            "psi": self.current_state.psi,
            "phi_rate": self.current_state.phi_rate,
            "theta_rate": self.current_state.theta_rate,
            "psi_rate": self.current_state.psi_rate,
            "airspeed": self.current_state.airspeed,
            "flight_mode": self.current_state.flight_mode,
            "systemArmed": self.current_state.armed,

            "throttle": self.control_input.fw.throttle,
            "aileron": self.control_input.fw.aileron,
            "elevator": self.control_input.fw.elevator,
            "rudder": self.control_input.fw.rudder,

            "Motor1": self.control_input.quad.motor1,
            "Motor2": self.control_input.quad.motor2,
            "Motor3": self.control_input.quad.motor3,
            "Motor4": self.control_input.quad.motor4,

            "lift": self.forces_moments.lift,
            "drag": self.forces_moments.drag,
            "Fx": self.forces_moments.fx,
            "Fy": self.forces_moments.fy,
            "Fz": self.forces_moments.fz,
            "l_moment": self.forces_moments.l,
            "m_moment": self.forces_moments.m,
            "n_moment": self.forces_moments.n,
        }

    def _log_header(self):
        return [
            "time", "x", "y", "z", "x_vel", "y_vel", "z_vel", "phi", "theta", "psi",
            "phi_rate", "theta_rate", "psi_rate", "airspeed", "flight_mode", "systemArmed",
            "throttle", "aileron", "elevator", "rudder",
            "Motor1", "Motor2", "Motor3", "Motor4",
            "lift", "drag", "Fx", "Fy", "Fz", "l_moment", "m_moment", "n_moment"
        ]

    def run_simulation(self):
        self.runsim = False
        log_path = Path("logger/logs")
        log_path.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = log_path / f"simulation_{timestamp}.csv"

        # Write CSV header only once
        with open(filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self._log_header())
            writer.writeheader()

            while True:
                self.GCS_data = self.interface.run()

                match self.GCS_data.sim_command:
                    case "START":
                        self.runsim = True
                        self.GCS_data.mode = "Auto"

                    case "PAUSE":
                        self.runsim = False
                        continue

                    case "RESET":
                        self.restart()
                        self.runsim = False
                        continue

                    case "STOP":
                        self.runsim = False
                        break

                if not self.runsim:
                    continue

                start_time = time.time()

                try:
                    self._simulate_step()
                    log_entry = self._generate_log_entry()
                    writer.writerow(log_entry)
                except Exception as e:
                    print(f"[ERROR] Simulation step failed: {e}")
                    self.runsim = False
                    continue

                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.freq) - elapsed)
                time.sleep(sleep_time)

if __name__ == "__main__":
    sim = UAVSimulator()
    sim.run_simulation()
