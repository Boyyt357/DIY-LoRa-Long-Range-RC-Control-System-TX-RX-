# **DIY LoRa Long-Range RC Control System (TX/RX)**

This project provides an open-source, affordable, and fully customizable remote control (RC) link solution utilizing readily available ESP32/ESP8266 microcontrollers and Semtech LoRa modules. By leveraging the superior sensitivity and interference resilience of LoRa modulation, this system is designed for long-range, hobbyist telemetry, and custom automation tasks where commercial systems might be overkill or lack configuration flexibility.

<img width="1200" height="800" alt="1c691774-9b0a-4870-8643-04b22585c6dc" src="https://github.com/user-attachments/assets/a0e5507d-cbf4-4bf2-bb60-68e9c9b6e2ab" />
# **https://youtu.be/mCVtn_sfCrs?si=kKAzGYDWoel_OTza**

## **🚀 Key Features**

| Feature | Description | Notes |
| :---- | :---- | :---- |
| **LoRa Modulation** | Utilizes Semtech's LoRa (Long Range) technology for excellent link budget and range capabilities, particularly at lower power outputs. | Ideal for ground vehicles, fixed-wing aircraft, and robotic platforms. |
| **Ultra-Low Cost** | Built using standard, mass-produced development boards (ESP32/ESP8266) and inexpensive LoRa transceiver modules (e.g., SX1276/SX1278). | Significantly cheaper than commercial TX/RX pairs. |
| **Full Customization** | The link is written in Arduino/C++, giving the user complete control over packet structure, refresh rate, channel count, and payload content. | Perfect for integrating non-standard sensor data (custom telemetry). |
| **Bi-Directional Link** | Supports simultaneous command transmission (TX to RX) and basic telemetry return (RX to TX), such as link quality (RSSI) and battery voltage. | Crucial for safety and monitoring the remote platform. |
| **High Channel Count** | Supports up to 16 channels, configurable via joystick inputs and auxiliary switches, matching standard hobby radio capabilities. | Output can be configured as PWM, PPM, or SBUS. |

## **🛠️ Installation and Setup Guide**

To build and run this system, you will need to assemble the hardware and flash the respective code to the Transmitter and Receiver microcontrollers.

### **1\. Hardware Required**

| Component | Quantity | Role |
| :---- | :---- | :---- |
| **MCU Board** | 2 | ESP32 or ESP8266 (e.g., NodeMCU, Wemos D1 Mini) |
| **LoRa Module** | 2 | SX1276, SX1278, or compatible (e.g., Ra-02) |
| **Analog Joystick** | 2-4 | For control axes (Pitch, Roll, Yaw, Throttle) |
| **Switches/Buttons** | 4+ | For Auxiliary channels (Arming, Modes, Lights) |
| **Power Supply** | 2 | Battery or regulated power supply for TX/RX |

### **2\. Wiring (SPI Connection)**

The ESP board communicates with the LoRa module using the SPI protocol. Connect the pins as follows (using common ESP32 pinouts as an example):

| LoRa Module Pin | ESP32 Pin (Typical) | Function |
| :---- | :---- | :---- |
| **VCC** | 3.3V | Power (Ensure your module is 3.3V compatible) |
| **GND** | GND | Ground |
| **SCK** | GPIO5 | SPI Clock |
| **MISO** | GPIO19 | Master In, Slave Out |
| **MOSI** | GPIO27 | Master Out, Slave In |
| **CS** | GPIO18 | Chip Select (SS) |
| **RST** | GPIO14 | Reset |
| **DIO0** | GPIO26 | Interrupt Pin (IRQ) |

### **3\. Software Setup (Arduino IDE)**

1. **Install Board Support:** Ensure you have the ESP32 or ESP8266 board support installed in your Arduino IDE.  
2. **Install Libraries:** Install the necessary LoRa library (e.g., the standard LoRa library by Sandeep Mistry).  
  
3. **Flash Firmware:** Upload the designated TX\_Firmware.ino to the Transmitter board and RX\_Firmware.ino to the Receiver board. Ensure the operating frequency (e.g., 433 MHz, 868 MHz, or 915 MHz) is set correctly in the code for your region.

## **🆚 Comparison to Cheap, Modern TX/RX Systems**

This custom LoRa link is the ultimate budget option, offering massive customization at the expense of standardized performance and ease-of-use. Below is a comparison to the two most popular modern, affordable alternatives: **ExpressLRS (ELRS)** and **FrSky R9**.

| Feature | This Custom LoRa System | ExpressLRS (ELRS) | FrSky R9 (ACCESS/ACCST) |
| :---- | :---- | :---- | :---- |
| **Core Technology** | ESP32 \+ LoRa (Raw Code) | ESP32/STM32 \+ LoRa (Optimized Protocol) | Proprietary 900 MHz (Uses LoRa in some modes) |
| **Cost (TX/RX Set)** | **Lowest ($15 \- $30 USD)** | Low ($30 \- $70 USD) | Medium ($80 \- $150 USD) |
| Max Update Rate | Low (\*\*\~50 Hz to 100 Hz\*\* typical) | **Extremely High (Up to 1000 Hz)** | Medium (150 Hz max) |
| **Latency** | Medium to High (10 ms \- 20 ms+) | **Ultra-Low (\< 4 ms)** | Medium (8 ms \- 15 ms) |
| **Range Potential** | Excellent (Limited by basic code optimization) | **Exceptional (Highly optimized for distance)** | Very Good |
| **Open Source** | 100% (Your code) | **Yes, fully open-source and community-driven.** | Partially/Closed (Proprietary protocol) |
| **Ease of Use** | Difficult (Requires C++ coding and soldering) | Easy (Plug-and-play modules, Wi-Fi updates, LUA scripts) | Easy (Commercial product) |
| **Binding/Setup** | Manual (Requires hard-coded IDs in the sketch) | Bind Phrase (Simple, standardized over-the-air binding) | Button/Model Match |

### ** Transmitter Wiring **
<img width="3044" height="1893" alt="transmitter" src="https://github.com/user-attachments/assets/0cf125c5-c24f-4e9e-b3cb-d72bfdb43508" />

### ** Receiver Wiring **
<img width="3055" height="1608" alt="Reciver" src="https://github.com/user-attachments/assets/d7cb2a0b-7d0f-4bce-a814-c2ed3e2f036d" />

### **Summary of Comparison**

* **Choose the Custom LoRa System if:** Your primary goal is to **learn** the underlying communication protocol, implement highly custom or non-standard payload data, or achieve the **absolute minimum hardware cost**.  
* **Choose ExpressLRS (ELRS) if:** You need **world-class, low-latency performance** (essential for FPV drones), plug-and-play ease of use, superior range optimization, and a standardized, commercially supported ecosystem, all while remaining highly affordable. ELRS is the undisputed champion of cheap and modern RC links.

## **💡 Use Cases**

This DIY LoRa system is best suited for projects where maximum throughput is less critical than simple reliability, affordability, and custom data handling.

1. **Ground Robotics / Rovers:**  
   * **Scenario:** Controlling a large R/C rover or tractor that needs to travel several kilometers away or operate in line-of-sight environments.  
   * **Benefit:** LoRa provides the necessary range and penetration for ground applications without needing the extremely low latency of FPV racing.  
2. **Long-Range Sensors & Telemetry:**  
   * **Scenario:** Monitoring environmental conditions (temperature, humidity, GPS location) on a remote, static platform, or a slow-moving fixed-wing aircraft.  
   * **Benefit:** The bi-directional link allows full customization of the telemetry packet to send back non-standard sensor data in parallel with the control signals.  
3. **Educational & Hobbyist Learning:**  
   * **Scenario:** Students or hobbyists learning embedded systems, SPI communication, and radio link design from the ground up.  
   * **Benefit:** The simplicity of the code base (direct LoRa.write() commands) makes it an excellent platform for understanding how digital radio control links actually work.  
4. **Custom Automation (Non-RC):**  
   * **Scenario:** Activating remote gates, controlling farm irrigation pumps, or triggering cameras long distance.  
   * **Benefit:** The system is easily adapted to send simple boolean or custom commands instead of standard 8-channel PWM data.
