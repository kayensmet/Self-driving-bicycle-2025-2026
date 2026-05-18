# Communication group 1 

Remote control system for an bicycle using ESP-NOW, a PS5 controller, MQTT, and Node-RED.
This communication strategy can be used for both bicycle as it has been tested on both bicycles (group 1 and 2) with a few adjustments, such as the pins on the esp32.

---

## Overview

Two ESP32s communicate wirelessly via **ESP-NOW**:

- **Sender** — connects to WiFi + MQTT, reads PS5 controller input via Bluepad32, sends data to receiver
- **Receiver** — gets data via ESP-NOW, drives DAC outputs (throttle/brake), controls lights and buzzer

```
PS5 Controller → ESP32 (Sender) ──ESP-NOW──► ESP32 (Receiver) → Bike actuators
                      │
                     MQTT
                      │
                 MQTT Broker
                      │
           (Node-RED) Dashboard
```

---

## Hardware

| Component | Role |
|---|---|
| ESP32 (Bluepad32 board) | Sender |
| ESP32 (standard DOIT DevKit) | Receiver |
| PS5 DualSense controller | Input |
| Local server | MQTT broker |

> **Note:** Type of controller or MQTT broker can be choosen differently. But because the code uses ESP-NOW mainly for communication you will need 2 esp32.

---

## Setup

### 1. Install Bluepad32 (Sender only)

Add this URL to Arduino IDE → Preferences → Additional Board Manager URLs:

```
https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
```

Select **"ESP32 + Bluepad32"** board for the sender. Use the standard ESP32 board for the receiver.

### 2. Find the receiver's MAC address

Flash a simple MAC-print sketch to the receiver and copy the address into the sender code:

```cpp
uint8_t receiverMAC[] = {0xE4, 0x65, 0xB8, 0x83, 0x66, 0xC4}; // update this
```

### 3. Configure WiFi credentials (Sender only)

```cpp
const char* ssid = "your-network";
const char* password = "your-password";
const char* broker = "your-broker-hostname-or-ip";
```

The receiver needs **no WiFi credentials**.

### 4. Match the WiFi channel

The receiver must use the same channel as the sender. The sender prints it on boot:

```
WiFi kanaal: 11
```

Set this in the receiver:

```cpp
const int WIFI_CHANNEL = 11; // match sender's channel
```

---

## MQTT & Node-RED

The sender publishes controller values as MQTT topics (for now we just have basic topics but you can choose your own topics, its designed to be flexible and easily adjusted):

| Topic | Value |
|---|---|
| `controller/l2` | Brake (0–1023) |
| `controller/r2` | Throttle (0–1023) |
| `espnow/zender` | Packet counter |

Run a broker (e.g. Mosquitto) on a local server (e.g. pi) and connect Node-RED to build a dashboard. Currently throttle and brake are visualised, easy to extend with:

- GPS coordinates → live map
- Battery voltage
- Steering angle
- Any other sensor on the bicycle (hall sensor, ...)

The goal: monitor and eventually fully control the bicycle remotely from a dashboard.

---

## Code Structure

Both sketches are intentionally simple and easy to modify.

**The shared data struct** is the main thing to change when adding new data:

```cpp
struct SensorData {
    int counter;
    float l2Volt;
    float r2Volt;
    bool r1Pressed;
    bool controllerConnected;
};
```

Add a field here on both sides and it's available everywhere.

**Switching away from PS5**, the Bluepad32 functions (`onConnectedController`, `processControllers`, etc.) are isolated. Swap them out for any other input source without touching the ESP-NOW or MQTT logic.

The code currently has verbose serial debug prints, useful for verifying packet delivery, WiFi/MQTT/controller connectivity, and DAC values.

---

## Current Status

- [x] PS5 → ESP-NOW → bicycle actuators (throttle, brake, lights, buzzer)
- [x] MQTT topics for live monitoring
- [x] Node-RED dashboard (throttle/brake)
- [ ] Extra features such as (GPS map in dashboard, Battery, sensors, ...)
- [ ] Safety PCB integrated
- [ ] ...
