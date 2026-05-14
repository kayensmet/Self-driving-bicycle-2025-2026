# Remote Controller System — IB3 Self-Driving Bicycle 2025–2026

## Remote Controller Design

To remotely control the bicycle, a custom remote controller was developed.

A stripboard PCB was used because the design mainly consists of separate modules connected through sockets.  
As a result, designing a custom PCB was not necessary.

The remote controller contains:

- 3 sliders
- 3 toggle switches
- TFT display
- ESP32

*(see figure)*

The remote communicates with all ESP32 modules in the system using **ESP-NOW**.

---

# Slider Controls

## Throttle and Brake

The two vertical sliders on the left side are used for:

- Throttle
- Brake

### Throttle Slider

The leftmost slider controls the throttle.

- `D32` → Throttle slider

The slider must be moved beyond approximately half its range before the bicycle starts moving.

> Be careful when operating the bicycle for the first time.

---

### Brake Slider

The second vertical slider controls the brake.

- `D33` → Brake slider

---

## Steering Slider

The horizontal slider on the right side controls the steering of the bicycle.

- `D34` → Steering slider

---

# Toggle Switches

Three toggle switches are available on the remote controller:

| Position | Function | ESP32 Pin |
|---|---|---|
| Left | Buzzer | `D14` |
| Middle | Balance system enable | `D12` |
| Right | Emergency stop | `D13` |

The emergency stop is currently **not implemented**, because the relay did not function reliably.

---

# TFT Display

To visualize the most important system feedback, a TFT display was added.

The display:
- operates at **3.3V**
- communicates via **SPI**
- contains **11 connection pins**

---

## TFT Display Connections

| TFT Pin | Connection |
|---|---|
| Lite | NC |
| SDCS | NC |
| DC | `D2` |
| RST | `D4` |
| TCS | `D5` |
| MOSI | `D23` |
| MISO | NC |
| SCK | `D18` |
| GND | Common ground |
| 3V | 3.3V pin of ESP32 |
| Vin | NC |

---

# Information Displayed on the Screen

The TFT display continuously shows the following information:

- Throttle value
- Brake value
- Steering direction
- Balance system status

---

## Throttle and Brake Feedback

The throttle and brake values shown on the display are the analog values read from the sliders.

---

## Steering Feedback

For steering, the display shows one of the following states:

- Left
- Right
- Straight

---

## Balance System Feedback

The display also indicates whether the balance system is:

- ON
- OFF

---

# Emergency Stop Screen

When the emergency stop switch is activated, a separate warning screen appears indicating that the emergency stop has been triggered.

While this screen is active:
- no controls on the remote can be used until the emergency stop is disabled again.

This functionality is currently only implemented on the remote controller side and not yet on the bicycle itself.
