# Hotzenblitz EV Re-Engineering Project ⚡🚗

This repository contains embedded software developed as part of the **Hotzenblitz Electric Vehicle Re-Engineering Project**, focusing on real-time motor control fundamentals.

## 📌 Project Overview
The objective of this project is to design and validate embedded control building blocks required for electric vehicle drivetrains, with emphasis on **PWM generation, ADC-based current sensing, and deterministic real-time sampling**.

## 🔧 Hardware Platform
- **Microcontroller:** TI C2000 F280039C LaunchPad
- **Clock Frequency:** 120 MHz
- **ADC:** 12-bit SAR ADC (ADCA / ADCB)
- **PWM:** ePWM modules
- **Sampling Rate:** 1 ms (1 kHz)

## 🧠 My Role
- Configured **ePWM as a deterministic timing source**
- Implemented **ePWM-triggered ADC sampling**
- Generated **independent PWM outputs**
- Performed ADC scaling and current computation
- Validated timing and signals using oscilloscope

## 🗂 Repository Structure
- `firmware/` – Embedded C source code (TI DriverLib)
- `docs/` – Block diagrams and timing diagrams
- `matlab_simulation/` – DSP and control simulations

## 🛠 Tools Used
- Code Composer Studio (CCS)
- TI C2000Ware & DriverLib
- MATLAB / Simulink
- Oscilloscope for validation

## 🚀 Future Work
- Digital filtering of ADC data
- Closed-loop current control
- CAN-based communication
- Inverter and motor integration

---
**Author:** Mohamed Ismail  
M.Sc Embedded & Autonomous Systems – WH Zwickau
