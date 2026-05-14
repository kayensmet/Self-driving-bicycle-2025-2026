# IB3 Self-Driving Bicycle — Project Summary 2025–2026: Summary of FINALIB3.PDF!



**KU Leuven Campus Gent**  
**Group 1:** Robin, Quinten, Maxime, Lars, Noah, Lee  
**Group 2:** Milan, Kayen, Thibaut, Toon, Simon V, Simon VK

---

## What is this project?

The goal of this project was to build a **fully autonomous self-driving bicycle** a real bike that can balance, drive, and steer itself without any physical human assistance. Two independent groups each built their own version of the bike and worked together on the shared balance system.

---

## The three core challenges

### 1. Balance
Keeping a bicycle upright without a rider is a classic control engineering problem. Both groups collaborated on this and went through two major approaches:

- **Plan A (Flywheel):** A spinning disc on the frame generating gyroscopic torque to counteract lean. Rejected after calculations showed the required motor speeds were physically unachievable at a reasonable cost.
- **Plan B (Drone motors):** Two BLDC drone motors with propellers mounted on a rod in the seat post. By varying thrust on each side, a corrective lateral force is generated. This worked.

Two control strategies were tested:
- **PID controller** briefly stable but failed due to motor inertia and derivative noise.
- **LQR controller** final solution. Uses state-space control to compute optimal corrections based on lean angle and angular rate. Achieved stable balancing within ±1° of target angle.

### 2. Drive
Both groups used hub motors in the rear wheel to propel the bike.

- **Group 1** used a Baserunner motor controller with DAC-based throttle control.
- **Group 2** used a Phaserunner V2 with a 3-phase hub motor on a 40V battery, controlled via throttle and brake signals.

Both groups encountered mechanical issues (axle slipping, Hall sensor errors) that limited full integration.

### 3. Steering
The front wheel steering was actuated by a servo motor.

- **Group 1** used a high-torque ASMC-05B servo (180 kg·cm) driving the fork through a gear system, later switching to a Capstan Drive to eliminate backlash.
- **Group 2** used a DS5160 servo with a chain and sprocket system after a belt drive failed due to slipping.

---

## Communication

- **Group 1** built a custom handheld controller using a PS5 DualSense over Bluetooth → ESP32 base station → ESP-NOW to the bike. Data was also published via MQTT and visualized on a live dashboard.
- **Group 2** built a custom physical controller (ESP32, linear slider potentiometers, TFT screen, buzzer) communicating via ESP-NOW with a heartbeat safety signal.

---

## Extra features (Group 2)

- Front light and brake light
- Buzzer for alerts
- Custom mid-frame panel and cable management
- Safety cutoff via controller

---

## What's still missing

- **Cornering:** the balance and steering systems are not yet coupled — the bike doesn't lean into corners
- **Autonomy:** currently fully remote-controlled, no object detection or path planning
- **Low-speed balance:** the bike struggles to stay upright at very low speeds or when stationary
- **Gain scheduling:** LQR gains are fixed regardless of speed; Hall sensor data could enable speed-dependent tuning
- **Single battery design** and consolidated PCB

---

## Repository structure

```
docs/          ← this website (GitHub Pages)
src/           ← all source code
  balance/     ← PID, LQR, dashboard
  steering/    ← servo control
  drive/       ← motor controller integration
assets/        ← presentation, datasheets
```
---

*IB3 — KU Leuven Campus Gent — Academic year 2025–2026*
