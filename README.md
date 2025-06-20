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

<div style="display: flex; align-items: flex-start; justify-content: space-between; gap: 20px;">

<!-- Left: Table -->
<div style="flex: 1;">
  
## Components

| Component         | Specs                             |
|------------------|-----------------------------------|
| Microcontroller   | MSP432E401Y @ 16MHz               |
| ToF Sensor        | VL53L1X, I2C, up to 4m range       |
| Stepper Motor     | 28BYJ-48, 2048 steps/rev           |
| Communication     | UART @ 115200 bps, I2C for sensor |
| Visualization     | Python, PySerial, Open3D          |

</div>

<div style="flex: 0 0 auto; text-align: centre;">
  <img src="https://github.com/user-attachments/assets/0fa3ba68-80b6-49c3-82b4-f089ae6acf95" height="400"/>
  <div><strong>Figure 1:</strong> Setup Image</div>
</div>

</div>

## Getting Started

1. Wire the components as described in the schematic.
2. Upload the C code to the MSP432 using Keil.
3. Install Python 3.9+ and required libraries:
   ```bash
   pip install open3d pyserial
4. Update the Python script with your COM port (e.g., `'COM4'`):

   Open the Python file and locate the following line:
   ```python
   s = serial.Serial('COM4', 115200)
5. In Keil:
   - Press **Translate**, then **Build**, and finally **Load** the project.
   - Press the **Reset** button on the board (next to the USB port) to flash the program.

6. In the Python script:
   - Update the `ROTATIONS` variable to match the number of revolutions you want the motor to complete (default is 3).
   - If changing the number of steps per rotation (default 32 for 11.25° intervals), also update:
     - `STEPS` in the Keil C file
     - `ANGLE = 360 / STEPS` in Python

7. Run the Python script. It writes the letter `'e'` to the UART interface, triggering the microcontroller. If LED1 (PN1) blinks, communication is successful.

8. Press the on-board button PJ1 to start scanning. The stepper motor will rotate and begin collecting data.

9. After each full revolution, LED3 (PF4) blinks—prompting the user to move forward ~600 mm for the next layer scan.

10. Once the full number of revolutions are completed, the collected data is processed and visualized in 3D using **Open3D**. A new window will open displaying the final model.

## 🖼️ Expected Output 

- Accurate 3D point cloud model of the scanned environment.
- Example: A hallway with varying width was successfully visualized, showing a narrow midsection and wider ends.
- The visual output clearly matches the real-world spatial characteristics.


<p align="center">
  <img src="https://github.com/user-attachments/assets/d1ef7cc8-810e-4bff-991c-49c142daf00b" height="500" alt="Hallway Start"/>

  <img src="https://github.com/user-attachments/assets/c4b1fa19-b3db-4a7d-b53d-d50798dbe8e6" height="500" alt="Side View"/>

</p>
<div align="center"><strong>Figure 2:</strong> Hallway Image</div>


<div align="center" style="flex: 0 0 auto; text-align: centre;">
  <img src="https://github.com/user-attachments/assets/92077979-8699-4f53-9549-289f92a375de" width="500" alt="Scan Result"/>
  <div><strong>Figure 3:</strong> Scan Image</div>
</div>



