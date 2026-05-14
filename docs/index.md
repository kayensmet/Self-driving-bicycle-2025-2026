# IB3 Self-Driving Bicycle 2025–2026

**KU Leuven Campus Gent**
## What's in this repo

### Documentation (this website)
Each section below has its own page explaining the approach, what worked, what didn't.

| Section | Description |
|---|---|
| [Balance](balance/index.md) | Drone motor system, PID, LQR controller |
| [Steering](steering/index.md) | Servo control, gear/chain drive |
| [Drive](drive/index.md) | Hub motor, motor controller, throttle|
| [Communication](communication/index.md) | ESP-NOW, MQTT, controller |

## Source Code

All source code is organized inside the `src/` directory at the root of this repository.

| Subsystem | Description | Source |
| :--- | :--- | :--- |
| **Balance** | PID controller, LQR controller, tuning dashboard | [Open folder](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/balance) |
| **Steering** | Servo control logic and steering actuation | [Open folder](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/steering) |
| **Drive** | Hub motor integration, throttle control, motor controller | [Open folder](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/drive) |
| **Communication** | ESP-NOW communication, base station, controller firmware | [Open folder](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/communication) |

## How to use this repo

If you are a new team member picking this up next year:

1. Read the [project summary](assets/summary.md) first for a full picture
2. Browse the section pages above for the details of each subsystem
3. Find the relevant source code in `src/`
4. Check the "What's missing" section on each page, and (hopefully) improve!

---
## Final product

<iframe width="560" height="315" src="https://www.youtube.com/embed/v913zEsBVsk" title="YouTube video speler" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
