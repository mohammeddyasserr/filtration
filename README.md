# 🚗 Vehicle Simulation with PID Controller (ROS 2)

This project demonstrates a simple **vehicle dynamics simulation** and a **PID speed controller** built on **ROS 2**.  
The system simulates a car that receives throttle/brake commands and attempts to reach and maintain a target speed (default: **60 km/h**).  

---

## 📌 Project Overview

- **vehicle.py** → Simulates the car’s motion.  
  - Input: `/cmd_vel` (Float32) → control signal in range `-1.0 … 1.0`.  
    - `+1.0` = full throttle  
    - `-1.0` = full brake  
  - Output: `/current_speed` (Float32) → car speed in **km/h**.

- **controller.py** → Implements a **PID controller**.  
  - Input: `/current_speed`  
  - Output: `/cmd_vel`  
  - Logs performance and saves results:  
    - 📊 `speed_plot.png` → Plot of target vs. actual speed over time.  
    - 📄 `speed_data.csv` → Logged time, actual speed, and target speed.

---

## ⚙️ How It Works

1. The **controller** sets a target speed (default: `60 km/h`).
2. The **vehicle node** simulates real-world motion:
   - Vehicle acceleration = `control × 5 − damping × speed`
   - Simple drag/damping factor applied.
   - Ensures speed never goes below 0.
3. The controller runs a **PID loop**:
   - Error = `(target speed − actual speed)`
   - PID calculates throttle/brake needed.
   - Output is clamped between `-1.0 … +1.0`.
