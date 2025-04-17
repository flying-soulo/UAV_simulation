# 6-DOF UAV Simulation – Quadplane

This project simulates a UAV’s flight in **6 Degrees of Freedom (6-DOF)** with modular architecture. It focuses on realistic physics, reusability, and future scalability.

> Designed for Model-Based Design using **ANSYS SCADE**. Supports multi-UAV platforms.

---

## 🚀 Project Features

- Modular structure supporting easy plug-and-play development
- Rigid-body dynamics with aerodynamic and propulsion modeling
- Realistic environment and sensor models
- Custom autopilot and control algorithm integration (PID, waypoint, etc.)
- Real-time data logging and visualization support

---

## 📁 Repository Structure

```plaintext
.
├── main.py                # Simulation entry point
├── utils.py               # Global utilities
├── config.json            # Configurable system parameters
├── requirements.txt       # Python dependencies
├── docs/
│   └── Simulation_Description.md   # Detailed design, architecture, and goals
├── tests/                 # Unit and integration tests
└── README.md              # You are here
```

## How to Use

1. Clone the repository:
    ```
    git clone <repository-url>
    cd <repository-folder>
    ```

2. Install dependencies:
    ```
    pip install -r requirements.txt
    ```

3. Run the application:
    ```
    python main.py
    ```