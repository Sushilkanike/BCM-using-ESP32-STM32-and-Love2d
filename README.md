# BCM-using-ESP32-STM32-and-Love2d
# Automotive Body Control Module (BCM) Prototype
### Integrated with STM32, ESP32, CAN Bus, and LÖVE2D

## 🚗 Project Overview
This project demonstrates the design and implementation of a custom **Body Control Module (BCM)** for automotive applications. It serves as a "hybrid" system that bridges robust, automotive-standard **CAN Bus** communication with modern, wireless visualization techniques.

The system acquires real-time vehicle sensor data (Speed, Fuel, Indicators) using an **STM32 microcontroller**, broadcasts it over a CAN network, bridges it to Wi-Fi via an **ESP32 gateway**, and visualizes the status on a custom PC dashboard built with the **LÖVE2D game engine**.

## 🏗 System Architecture
The project is divided into three distinct nodes forming an end-to-end data pipeline:

1.  **Acquisition Node (The "Car" - STM32F401):** * Reads physical sensors (Potentiometer, MPU6050, Buttons).
    * Processes data (Debouncing, ADC conversion).
    * Transmits data via CAN Bus (MCP2515).
2.  **Gateway Node (The Bridge - ESP32):**
    * Sniffs the CAN Bus for specific IDs (0x100, 0x101).
    * Converts CAN frames into UDP datagrams.
    * Broadcasts data over Wi-Fi.
3.  **Visualization Node (The Dashboard - LÖVE2D):**
    * Listens for UDP packets.
    * Parses strings (e.g., `SPEED:60;FUEL:75`).
    * Updates a graphical dashboard in real-time.

![System Diagram](path/to/your/images/system_diagram.png)
*Figure 1: Top Level System Diagram*

## 🛠 Hardware & Software

### Hardware Required
* **STM32F401CCU6 "Blackpill"** (Core BCM)
* **ESP32 Wroom-32E** (Wi-Fi Gateway)
* **MCP2515** CAN Bus Modules (x2)
* **MPU6050 (GY-87)** Accelerometer/Gyro
* **16x2 I2C LCD** Display
* **Slide Potentiometer** (Simulating Fuel Level)
* **Tactile Pushbuttons** (x6)
* Breadboards and Jumper Wires

### Software Tools
* **STM32CubeIDE:** For STM32 firmware (C, HAL APIs).
* **Arduino IDE:** For ESP32 gateway programming.
* **LÖVE2D:** For the Dashboard application (Lua).
* **Packet Sender:** For network debugging.

## 🔌 Pin Configuration (STM32)

| Component | STM32 Pin | Function |
| :--- | :--- | :--- |
| **Potentiometer** | PAx (ADC1) | Analog Input (DMA) |
| **Button 1** | PB0 | Left Indicator (EXTI) |
| **Button 2** | PB1 | Right Indicator (EXTI) |
| **Button 3** | PB10 | High Beam (EXTI) |
| **Button 4** | PB3 | Check Engine (EXTI) |
| **Button 5** | PB4 | Check Oil (EXTI) |
| **Button 6** | PB5 | Battery Warning (EXTI) |
| **CAN Module** | SPI1 Pins | MCP2515 Communication |
| **MPU6050** | I2C1 Pins | Accelerometer Data |
| **LCD** | I2C1 Pins | Local Status Display |

*> **Note:** Buttons use hardware interrupts (EXTI) with software debouncing (200ms).*

## 🚀 Installation & Setup

### 1. STM32 Firmware
1.  Open the `STM32_Firmware` folder in **STM32CubeIDE**.
2.  Build the project to generate the `.elf` / `.bin` file.
3.  Flash the code to the STM32F401 using an ST-Link debugger.
4.  *Key Feature:* The firmware uses DMA for the ADC to reduce CPU load and a Super-loop architecture for sequential logic.

### 2. ESP32 Gateway
1.  Open the `ESP32_Gateway` sketch in **Arduino IDE**.
2.  Update the `SSID` and `PASSWORD` variables to match your local Wi-Fi.
3.  Upload the code to the ESP32.
4.  Monitor the Serial output to verify it connects to Wi-Fi and initializes the CAN module.

### 3. Dashboard Application
1.  Install [LÖVE2D](https://love2d.org/).
2.  Navigate to the `Dashboard_Lua` folder.
3.  Run the application (drag the folder onto the `love.exe` or run via command line).
4.  The dashboard will listen on the configured UDP port.

## 📡 Communication Protocol

**CAN Bus Strategy:**
* **ID 0x100:** Sends Fuel Level (Byte 0) and Button States (Byte 1).
* **ID 0x101:** Sends MPU6050 Accelerometer Data (X, Y, Z axis).

**UDP Packet Format:**
The ESP32 converts CAN data into a string format for the dashboard:
```text
"SPEED:22.8;FUEL:84;GEAR:D;BLINK:LEFT;..."
