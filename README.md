# 🚙 ESP32 4WD RC Car (BTS7960 + Bluetooth)

This project is a **DIY 4-wheel drive robotic car** powered by an **ESP32**, **BTS7960 motor drivers**, and Bluetooth communication.  
The ESP32 handles motor control and receives commands from a smartphone app over Bluetooth.

---

## ✨ Features
- 🚗 Forward, backward, left, right movement  
- ⚡ High-current motor control with BTS7960 drivers  
- 📱 Controlled via smartphone Bluetooth app  
- 🔋 Supports higher motor loads compared to L298N setups  
- 🌐 ESP32 ready for Wi-Fi upgrades (MQTT, web control, etc.)  

---

## 🛠️ Hardware Requirements
- ESP32 Dev Board  
- 2 × BTS7960 motor driver modules (one for left motors, one for right motors)  
- HC-05/HC-06 Bluetooth module **(or ESP32 built-in Bluetooth)**  
- 4 DC Motors + 4WD chassis  
- High-capacity battery pack (11.1V–12V recommended)  

---

## 💻 Software Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support  
- Bluetooth RC Controller app (from Google Play Store)  

---

## 🔌 Wiring Diagram (Example)
| ESP32 Pin   | BTS7960 Pin | Function        |
|-------------|-------------|-----------------|
| GPIO 25     | RPWM (Left) | Left motor PWM  |
| GPIO 26     | LPWM (Left) | Left motor PWM  |
| GPIO 27     | RPWM (Right)| Right motor PWM |
| GPIO 14     | LPWM (Right)| Right motor PWM |
| 5V (VIN)    | VCC         | Logic power     |
| GND         | GND         | Common ground   |

⚠️ **Note:** Use a separate power source for motors (12V) and connect grounds together.  


