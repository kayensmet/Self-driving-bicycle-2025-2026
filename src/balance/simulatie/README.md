# Inverted Pendulum PID Simulation

- Clone and open the project in CLion.
- Real-time inverted pendulum simulator built with C++, Dear ImGui and DirectX11.


## Requirements
- Windows 10/11
- CLion (with CMake support)
- Visual Studio Build Tools (MSVC)
- DirectX 11 SDK (included in Windows SDK)

## Build
Open the project in CLion and let CMake configure automatically, then build and run.

## Features

- Real-time PID controller simulation
- Adjustable physical parameters
- Live visualization of the pendulum system
- Real-time graphs for:
  - P term
  - I term
  - D term
  - Motor 1 and 2 output
  - Pendulum angle


---

# Adjustable Parameters

The simulator allows live modification of:

- Pendulum weight
- Motor thrust / force
- Rod length
- Center of gravity
- Initial angle

---

# PID Controller

Live tuning of:
- `Kp`
- `Ki`
- `Kd`

---

# Technologies

- C++
- Dear ImGui
- DirectX11
- Win32 API
- CMake
- CLion

---

# Real-Time Graphs

The application visualizes:
- Proportional response
- Integral accumulation
- Derivative damping
- System stability
- Oscillation behavior
- Overshoot

---

# Simulation demo
You can see the angle of the pendulum. The output for the two motors. P, I and D values.

![Simulatie voorbeeld](sim_voorbeeld.gif)

