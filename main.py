import time
import numpy as np
import threading
import pandas as pd
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from Autonomy.Autopilot import Autopilot_class
from AeroVehicle.Vehicle_Sim import Aero_Vehicle_sim
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Globals
sim_thread = None
latest_state = {}
sim_log = []

# Config
freq = 100
dt = 1 / freq

# Setup
waypoints = [
    [+5000, +5000, -1000, 30, "reach"],
    [+5000, -5000, -1000, 30, "reach"],
    [-5000, -5000, -1000, 30, "reach"],
    [-5000, +5000, -1000, 35, "reach"],
    [+5000, +5000, -1000, 35, "reach"],
]

vehicle_prop = Aerosonde_vehicle.copy()
simulation = Aero_Vehicle_sim(vehicle_prop, dt)
autopilot = Autopilot_class(waypoints, dt)

# Initial state
current_state = np.zeros(12)
current_state[2] = -1000
current_state[3] = 23


def run_step(state):
    thrust_max = 110 * 0.3
    thrust_min = 0
    deflection_max = np.deg2rad(30)
    deflection_min = np.deg2rad(-30)

    motor_thrust, ctrl_srfc_deflection = autopilot.run(state)
    motor_thrust = np.clip(motor_thrust, thrust_min, thrust_max)
    ctrl_srfc_deflection = np.clip(ctrl_srfc_deflection, deflection_min, deflection_max)

    return simulation.simulate_one_step(state, motor_thrust, ctrl_srfc_deflection)


def record_log(t, state):
    sim_log.append({
        'time': round(t, 2),
        'x': round(state[0], 2),
        'y': round(state[1], 2),
        'z': round(state[2], 2),
        'u': round(state[3], 2),
        'v': round(state[4], 2),
        'w': round(state[5], 2),
        'phi': round(np.rad2deg(state[6]), 2),
        'theta': round(np.rad2deg(state[7]), 2),
        'psi': round(np.rad2deg(state[8]), 2),
    })


def update_visuals(state):
    global latest_state
    latest_state = {
        'x': round(state[0], 2),
        'y': round(state[1], 2),
        'z': round(state[2], 2),
        'phi': round(np.rad2deg(state[6]), 2),
        'theta': round(np.rad2deg(state[7]), 2),
        'psi': round(np.rad2deg(state[8]), 2),
    }
    socketio.emit('telemetry', latest_state)


def simulation_loop():
    global current_state
    t = 0.0

    try:
        while True:
            start_time = time.time()

            current_state, _ = run_step(current_state)
            record_log(t, current_state)
            update_visuals(current_state)

            t += dt
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Simulation interrupted. Saving log.")
    finally:
        df = pd.DataFrame(sim_log)
        df.to_csv("simulation_log.csv", index=False)
        print("Simulation log saved to simulation_log.csv")


@app.route("/")
def index():
    return render_template("viewer.html")


@socketio.on('connect')
def handle_connect():
    print("Client connected.")


@app.route("/start")
def start_sim():
    global sim_thread
    if sim_thread is None or not sim_thread.is_alive():
        sim_thread = threading.Thread(target=simulation_loop)
        sim_thread.start()
    return "Simulation started."


if __name__ == "__main__":
    socketio.run(app, debug=True)
