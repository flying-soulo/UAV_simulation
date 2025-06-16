# UAV Simulation Project

**Author:** Abhishek P M \
**Last Updated:** June 2025



## 🎯 Objectives

- Simulate UAV dynamics in **6 Degrees of Freedom (6-DOF)**
- Follow a **modular, scalable architecture** for multi-platform support
- Enable development and testing of **control algorithms** (e.g., PID, Guidance)
- Provide support for **real-time visualization and data logging**
- Allow **plug-and-play development** for controllers, sensors, and missions



## 🧭 Purpose

This simulation framework provides a structured, high-fidelity environment for the development and testing of UAV systems. It allows early-stage prototyping of guidance, navigation, and control (GNC) strategies in software before hardware deployment.


## 🏗️ System Architecture

The architecture is modular and decoupled across distinct functional domains:


Each module interfaces via a standardized shared data format, allowing independent upgrades and minimal coupling.


## 🧩 Module Breakdown

### 🚁 AeroVehicle/
- **Kinematics.py**: Updates UAV pose and velocity using body rates
- **Dynamics.py**: Implements Newton-Euler-based rigid body dynamics
- **Vehicle_Sim.py**: Aggregates physics modeling, runs one full sim step
- **Vehicle_Properties.py**: UAV-specific mass and inertia parameters

### 🧠 Autonomy/
- **Controller.py**, **PID.py**, **fw_controller.py**, **quad_controller.py**: UAV-specific control loops
- **Autopilot.py**, **guidance.py**, **path_planning.py**: High-level mission and waypoint logic
- **FMM.py**: yet to implement the FMM and auto navigation
- **Trim.py**: Computes trim conditions for steady-level flight, used in previous version - currently not used

### 🖥️ GUI/
- **interface.py**: GUI interface establishes a vpython windows with rendering and input handling
- **renderer.py**: renders the 3d environment, UAV model and it's location and orientation along with telemetry data
- **aircraft.py**: renders a UAV model in vpython environment
- **environment.py**: creates environmental effects to get a relative sense of the aircraft moevements
- **input_handler.py**: creates a input interfaces and upates data in real time

### 🛠️ Global/
- **configs.py**: Centralized configuration settings - yet to implement all the configs, most configs are defined locally
- **simdata.py**: contians dataclasses used in the whole project, allowing to track and manage the modules interaction with each other
- **utils.py**, **filter.py**: Math utilities and sensor filtering

### 📊 logger/
- **datalogger.py**: CSV-based time-series logging
- **sim_plot.py**: Matplotlib-based plotting utility
- **Outputs**: `simulation.csv`, `simulation.png`
    Note: developed for previous version not incorporated in this version



## 🔄 Simulation Flow

The simulation integrates through `main.py`, which:

1. Initializes vehicle simulation, Autopilot, and GUI
2. Iteratively updates physics, control, and GUI
3. Logs data and triggers visualization

All data is exchanged through the dataclass objects defined in `Global/simdata.py`, ensuring synchronization across modules.


## 🚀 Key Features

- ✅ 6-DOF Rigid Body Dynamics
- ✅ Modular support for **fixed-wing and multirotor UAVs**
- ✅ Plug-and-play controller design (PID, Guidance)
- ✅ Integrated **3D GUI visualization**
- ✅ Logging and real-time plotting
- 🔄 Implemention test modules for each modules - only available for GUI


## 📁 How to Run

```bash
# Install dependencies
pip install -r requirements.txt

# Run the simulation
python -m main
