# 🤖 Self-Balancing Robot (STM32 + MPU9250 + PID)

A modular, STM32-based **self-balancing robot** that uses an **IMU sensor** (MPU9250), **PID control**, and a safety framework to maintain upright stability.
This project demonstrates how to build robotics software with a **clean architecture** — separating hardware drivers, control logic, and safety mechanisms into dedicated modules.

---

## 🚀 Features

* **Modular Architecture**
  Clean separation of responsibilities:

  * `MotorController` → handles H-bridge motor drivers
  * `IMUSensor` → reads pitch/roll from MPU9250
  * `PIDController` → stabilizes robot with tunable PID gains
  * `SafetyManager` → emergency stop, tilt limits, and failsafes
  * `CommandHandler` → serial command interface

* **Real-Time Balance Control**
  Uses a **complementary filter** for sensor fusion and **PID loop** for stability.

* **Safety First**
  Automatic **emergency stop** if tilt exceeds safe range or if motor output is unstable.

* **Interactive CLI**
  Control robot via serial monitor:

  * `start` → enable balancing
  * `stop` → disable motors
  * `set Kp/Ki/Kd` → tune PID values
  * `status` → print system state

---

## 🛠️ Hardware

* **STM32F401CCU6** (Black Pill) – main controller
* **MPU9250** – 9-axis IMU (accelerometer + gyroscope)
* **L298N / L293D** – H-bridge motor driver
* **DC Motors + Wheels** – drive system
* **Battery Pack** – 7.4V Li-Po or similar

---

## ⚙️ Control Logic (High-Level)

1. **Read Orientation** → `IMUSensor.getPitch()`
2. **Compute PID Output** → `PIDController.compute(error)`
3. **Validate Safety** → `SafetyManager.isSafe(pitch, correction)`
4. **Drive Motors** → `MotorController.drive(correction)`

---

## 🔧 Setup & Usage

### 1. Install dependencies

* STM32 Arduino Core (via Arduino IDE Board Manager)
* MPU9250 library (e.g., `MPU9250.h`)

### 2. Flash firmware

* Open `sketch_oct1a.ino` in Arduino IDE or PlatformIO
* Select **STM32F401CCU6** as target board
* Upload via USB

### 3. Run

* Power the robot on a flat surface
* Open Serial Monitor (`115200 baud`)
* Send `start` → robot attempts to balance

---

## 🧪 Tuning PID

Use commands via Serial Monitor:

```
set Kp 10.5
set Ki 0.3
set Kd 2.1
```

Adjust until robot balances smoothly with minimal oscillation.

---

## 📊 Future Improvements

* [ ] Add Bluetooth app for wireless control
* [ ] Implement Kalman filter for better sensor fusion
* [ ] Auto-tuning for PID parameters
* [ ] Logging system for IMU & motor data

---

## 📜 License

This project is open-source. Feel free to use, modify, and share.
