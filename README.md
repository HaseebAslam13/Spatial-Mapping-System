# 3D LiDAR Scanner with MSP432 and Python Visualization

This project implements a low-cost 3D LiDAR scanning system using the **MSP432E401Y SimpleLink Microcontroller**, **VL53L1X Time-of-Flight sensor**, and a **28BYJ-48 stepper motor**. The system captures spatial distance data and reconstructs a 3D visualization using **Open3D in Python**.

## Features

- 📡 **VL53L1X ToF Sensor**: Measures distances up to 4 meters via I2C.
- ⚙️ **Stepper Motor**: 2048 steps/rev, rotates in increments of 11.25° to collect spatial data.
- 💡 **LED Feedback**: Visual indicators for UART connection, motor rotation, and sensor reads.
- 🧠 **MSP432 MCU**: Handles motor control, data acquisition, and communication with PC.
- 🖥️ **Python Visualization**: Uses Open3D to render 3D point clouds from distance data.

## How It Works

1. PC sends a start command ('e') to the microcontroller via UART.
2. User presses an onboard button to begin scanning.
3. The stepper motor rotates, and the ToF sensor collects distance readings at 32 angles per revolution.
4. After each full rotation, the user steps forward 600 mm.
5. All data is sent to the PC in a batch and visualized using Python.

## Components

| Component         | Specs                             |
|------------------|-----------------------------------|
| Microcontroller   | MSP432E401Y @ 16MHz               |
| ToF Sensor        | VL53L1X, I2C, up to 4m range       |
| Stepper Motor     | 28BYJ-48, 2048 steps/rev           |
| Communication     | UART @ 115200 bps, I2C for sensor |
| Visualization     | Python, PySerial, Open3D          |

## Getting Started

1. Wire the components as described in the schematic.
2. Upload the C code to the MSP432 using Keil.
3. Install Python 3.9+ and required libraries:
   ```bash
   pip install open3d pyserial

![Screenshot 2025-04-02 143437](https://github.com/user-attachments/assets/92077979-8699-4f53-9549-289f92a375de)
![shared image (19)](https://github.com/user-attachments/assets/d1ef7cc8-810e-4bff-991c-49c142daf00b)
![shared image (20)](https://github.com/user-attachments/assets/c4b1fa19-b3db-4a7d-b53d-d50798dbe8e6)
![shared image (21)](https://github.com/user-attachments/assets/0fa3ba68-80b6-49c3-82b4-f089ae6acf95)
