from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from AeroVehicle.Vehicle_Sim import UAVSimulation
from Autonomy.Autopilot import UAVAutopilot
from AeroVehicle.Vehicle_Properties import Aerosonde_vehicle

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Configuration
freq = 100
dt = 1 / freq
vehicle_prop = Aerosonde_vehicle.copy()

waypoints = [
    [+5000, +5000, -1000, 30, "reach"],
    [+5000, -5000, -1000, 30, "reach"],
    [-5000, -5000, -1000, 30, "reach"],
    [-5000, +5000, -1000, 35, "reach"],
    [+5000, +5000, -1000, 35, "reach"],
]

# Initialize simulation and autopilot
simulation = UAVSimulation(vehicle_prop, dt)
autopilot = UAVAutopilot(waypoints, dt)


# Simulation control
simulation_running = False

@app.route("/")
def index():
    return render_template("viewer.html")

@socketio.on('connect')
def handle_connect():
    print("Client connected.")

@socketio.on('start_simulation')
def start_simulation():
    global simulation_running
    if not simulation_running:
        simulation_running = True
        socketio.start_background_task(target=simulation_loop)
    emit('simulation_status', {'status': 'started'})

@socketio.on('stop_simulation')
def stop_simulation():
    global simulation_running
    simulation_running = False
    simulation.save_log()
    emit('simulation_status', {'status': 'stopped'})

def simulation_loop():
    global simulation_running
    while simulation_running:
        motor_thrust, ctrl_srfc_deflection = autopilot.compute_control(simulation.current_state)
        state = simulation.run_step(motor_thrust, ctrl_srfc_deflection)
        telemetry = {
            'x': round(state[0], 2),
            'y': round(state[1], 2),
            'z': round(state[2], 2),
            'phi': round(np.rad2deg(state[6]), 2),
            'theta': round(np.rad2deg(state[7]), 2),
            'psi': round(np.rad2deg(state[8]), 2),
        }
        socketio.emit('telemetry', telemetry)
        socketio.sleep(dt)

if __name__ == "__main__":
    socketio.run(app, debug=True)
