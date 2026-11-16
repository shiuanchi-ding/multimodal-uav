# Multimodal UAV for Locating Victims in Concealed Spaces

A vehicle that both flies and drives. When a space is too tight to fly through, it lands, crosses on wheels and takes off again.

Capstone Project, National Central University (Feb 2025 – Jan 2026) · NSTC-funded
Co-advised by Profs. Y.-T. Lin, P.-C. Tung and C.-K. Lin

- **Project page:** https://shiuanchi-ding.github.io/projects/uav.html
- **Full report (PDF):** https://shiuanchi-ding.github.io/files/uav-capstone.pdf
- **Paper, iFUZZY & FUZZY 2025 (PDF):** https://shiuanchi-ding.github.io/files/uav-paper.pdf

## Overview

- **Mechanism:** one servo per arm rotates it between a propeller and a wheel configuration, and a slider-crank support lifts the body during the change. FEA was run on the wheel and the arm.
- **Ground-mode model:** Simulink model of the drive motors (voltage and torque equations, measured speed curves) and a tire slip model.
- **Flight-mode model:** quadrotor model with PID attitude control whose gains are tuned online by model-reference adaptive control (MRAC). Position settles in about 3 s in simulation.
- **Embedded system:** STM32 with an IMU (Kalman-filtered attitude), a calibrated magnetometer, an altimeter and a Bluetooth phone app.
- **FPGA:** UART and I²C interfaces on a Zybo Z7-20 (Verilog, Vivado, Vitis).
- **Test rigs:** thrust rig, motor-speed rig and a universal-joint flight platform.

## My role

Team of 4. I set the development roadmap, designed the transformable mechanism and all test rigs, built the ground-mode Simulink model and the mathematical flight model, modeled the flight attitude dynamics with a teammate, did all of the FPGA work, and gave the oral presentation at iFUZZY & FUZZY 2025 (Honorable Mention).

## Tools

SolidWorks · MATLAB/Simulink · STM32 · Verilog · Vivado / Vitis

## Contact

Shiuan-Chi Ding · curryding0905@gmail.com · https://shiuanchi-ding.github.io
