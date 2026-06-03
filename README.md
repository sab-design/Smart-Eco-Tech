# Smart Eco-Tech: Embedded Audio Automation & Environment Monitor

An advanced embedded systems project using the **Tiva C Series (TM4C123G) LaunchPad**. This project tracks ambient temperature and humidity while using a Fast Fourier Transform (FFT) algorithm to filter audio frequencies and trigger motor sequences.

## 🚀 Features
* **Acoustic Automation:** Continuous FFT processing detects a specific 1200 Hz frequency component to automate an emergency motor response.
* **Timed Motor Sequences:** Runs an automated pattern (3s Forward -> 1s Break -> 3s Reverse) upon audio threshold trigger.
* **Environment Logging:** Real-time collection and monitoring of room temperature and humidity on an OLED interface.

## 🔌 Hardware Connections

| Component | Sensor Pin | Tiva C LaunchPad Pin |
| :--- | :--- | :--- |
| **Microphone** | OUT | `PE3` (AIN0) |
| **DHT11 Sensor** | DATA | `PD2` (Requires 10k Pull-up) |
| **OLED Display** | SCL / SDA | `PE4` / `PE5` (I2C2) |
| **Motor Driver** | IN1 / IN2 | `PD0` / `PD1` |

*Note: Connect the GND of Tiva C, Motor Driver, and the External Battery together to establish a **Common Ground**.*

## 🛠️ Software Stack
* **Language:** C++ / C (TivaWare Driver Libraries)
* **Core Algorithms:** 128-point Radix-2 Decimation-in-Frequency FFT
