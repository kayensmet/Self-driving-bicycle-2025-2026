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

### Source code
All code is organized in the `src/` folder at the root of this repository:

| Folder | Description |
|---|---|
| [Balance](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/balance) | PID controller, LQR controller, tuning dashboard |
| [Steering](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/steering) | Servo control logic |
| [Drive](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/drive) | Motor controller integration |
| [Communication](https://github.com/kayensmet/Self-driving-bicycle-2025-2026/tree/main/src/communication) | ESP-NOW, base station, controller firmware |
---

## How to use this repo

If you are a new team member picking this up next year:

1. Read the [project summary](assets/summary.md) first for a full picture
2. Browse the section pages above for the details of each subsystem
3. Find the relevant source code in `src/`
4. Check the "What's missing" section on each page, and (hopefully) improve!

---
