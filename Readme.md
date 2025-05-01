# 6-DOF UAV Simulation – Quadplane

**Author:** Abhishek P M
**Date:** March 27, 2025

---

## 🎯 Objectives

- Simulate UAV flight in **6 Degrees of Freedom (6-DOF)**
- Follow a **modular architecture** for flexibility and scalability
- Support **multiple UAV configurations** and future upgrades
- Implement **realistic physics modeling**, including aerodynamics and propulsion
- Enable **plug-and-play** integration of controller algorithms (e.g., PID, waypoint navigation)
- Provide **sensor and environmental models**
- Support **real-time data logging** for analysis and visualization

---

## 🧭 Purpose

This project provides a structured, high-fidelity simulation platform for UAV system development and testing. It allows verification of flight dynamics, control strategies, and sensor interactions in software—eliminating early dependency on hardware prototypes.

---

## ⚙️ Assumptions

- UAV is modeled as a **rigid body**
- Simulation follows **model-based design principles**
- Does not rely on any specific proprietary toolchain

> Originally designed with **ANSYS SCADE** compatibility in mind. Can be extended to support multi-UAV platforms.

---

## 🏗️ System Architecture

The system adopts a **modular and decoupled architecture**. Each module is independently upgradable and interacts with others using a **standardized data structure**. This ensures clean separation of concerns and promotes maintainability.

---

## 🧩 Module Overview

### 1. **Simulation Module**
- `plant`: Simulated UAV physical model
- `kinematics`: Updates position, velocity, and orientation
- `dynamics`: Applies Newton-Euler equations using force/torque inputs

### 2. **Autopilot Module**
- `autopilot`: High-level mission logic
- `waypoint`: Navigation and path planning
- `controller`: Abstract control interface
- `PID`: Classical control implementation with tunable parameters

### 3. **Data Logging Module**
- `simulation_log`: Time-series data storage
- `simulation_plot`: Post-run plotting and visualization

### 4. **Visualization Module**
- `virtual`: 2D/3D visual interface
- `models`: Geometric representations of UAVs

### 5. **Math Module**
- `utils`: Shared utility functions (I/O, transformations, etc.)

---

## 🔄 Integration Method

A central **Simulation and Integration Layer** synchronizes all modules through a **uniform struct-based data exchange format**.

Benefits:
- Easy replacement or upgrade of components
- Clean and testable architecture
- Promotes rapid prototyping and debugging

---

## 🚀 Project Features

- Fully modular, extensible framework
- Rigid-body dynamics with realistic force modeling
- Sensor and environment simulation
- Customizable control logic (PID, waypoint-following, etc.)
- Real-time logging and 3D visualization support

---

## 📁 Repository Structure

