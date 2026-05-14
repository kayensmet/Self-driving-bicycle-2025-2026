# Balance System — IB3 Self-Driving Bicycle 2025–2026

**Team:** Group 1 (Robin, Quinten, Maxime, Lars, Noah, Lee) & Group 2 (Milan, Kayen, Thibaut, Toon, Simon V, Simon VK)  
**Note:** Both groups collaborated on the balance system.

For full calculations, slides, and detailed comparisons, refer to the [final presentation (PDF)](../assets/FinalIB3.pdf).

---

## Contents

1. [Problem statement](#problem-statement)
2. [Plan A — Flywheel](#plan-a--flywheel)
3. [Plan B — Drone motors](#plan-b--drone-motors)
4. [Sensors](#sensors)
5. [PID controller](#pid-controller)
6. [LQR controller](#lqr-controller)
7. [What's missing for next year](#whats-missing-for-next-year)

---

## Problem statement

The goal is a bicycle that stays upright and moves without any physical human assistance. The balance system is responsible for keeping the bike from falling over at all times, whether stationary or in motion.

---

## Plan A — Flywheel

The first approach was a flywheel mounted on the bike frame (idea originated from: [Youtube Video](https://www.youtube.com/watch?v=2Z67NkvXIF4). The idea: spinning a heavy disc and varying its speed creates a gyroscopic reaction torque that counteracts the bike's lean angle.

After detailed calculations (see presentation, slides 10–12), both a solid disc and a hollow ring configuration were evaluated across three tilt scenarios (5°, 10°, 15°) with different centers of gravity. Both configurations failed the required angular accelerations were physically unachievable with any reasonably priced motor, and the exotic actuators that could deliver the torque weren't fast enough and super expensive for a prototype bicycle.

---

## Plan B Drone motors 

The second approach uses two BLDC drone motors mounted on a rod in the seat post, each with a propeller. By varying the thrust of each motor, a lateral corrective force is generated at a high moment arm. This moment arm can be changed, making it future proof for more power/less power thats needed to counteract the bike's lean.

**Why this works:**
- Variable moment arm gives design tolerance
- Motor cost is roughly 1/10th of Plan A
- Early tests showed an 800kV motor could already straighten the bike without a battery

**Downsides to be aware of:**
- Non-linear relationship between motor RPM and thrust (fluid dynamics)
- Safety risk from spinning propellers — a protective cage was 3D-printed around them (see presentation)

**Dimensioning result:** at 32 kg total bike mass, a center of gravity at 0.75 m, and a maximum tilt of 10°, each motor needs to deliver approximately 1.4 kg equivalent thrust. Full calculations are in the presentation (slides 19).

---

## Sensors

The **MPU6050** IMU (gyroscope + accelerometer) was used to measure the lean angle of the bike.

Two setups were tested:
- Averaging two MPU6050s: σ = 0.161° — usable but noisy
- Kalman filter: σ = 0.020° — much cleaner but introduced too much delay

The final implementation used a **complementary filter** as a compromise between noise and latency. The sensor was also moved from a loose wooden board to custom brackets directly on the frame to reduce vibration interference.

---

## PID controller

A classic PID controller was the first control strategy attempted. The IMU reads the lean angle, the PID computes a correction, and the output is mapped to motor throttle via DSHOT.

It showed **brief moments of stability** but ultimately failed due to:
- Physical inertia of the motor/propeller system being too high
- Measurement noise amplified by the derivative term causing positive feedback loops

The PID source code can be found in `src/balance/pid/`.

---

## LQR controller

After PID proved insufficient, an **LQR (Linear Quadratic Regulator)** was implemented using the `AutoLQR` library. This was the final and best-performing control strategy. [(Library)](https://github.com/lily-osp/AutoLQR)

### How it works

LQR is a state-space controller. Instead of manually tuning P, I and D terms, it computes an optimal gain matrix **K** that minimizes a cost function balancing correction effort against control energy:

```
u = -K · x
```

The state vector is: `x = [angle error, rate error]`
- Angle error = target angle − current pitch
- Rate error = 0 − gyro rate

The `AutoLQR` library takes a simplified system model and three tuning parameters, then automatically solves the Riccati equation to find the optimal K. This makes it far more robust than PID.

### Tuning parameters

| Parameter | Effect |
|---|---|
| Q angle | Higher = stronger reaction to angle error |
| Q rate | Higher = more damping on angular velocity |
| R control | Higher = smoother, less aggressive control |
| MotorBase | Idle throttle to keep motors spinning |
| MotorTrim | Compensates thrust difference between motor 1 and 2 |

### Final settings (Group 2)

- Target angle: −1.1°
- Q angle: 30 / Q rate: 1.5 / R control: 0.1
- Model damping: 0.98 / Control scale: 35
- Motor base: 240 / Motor trim: 0

### Result

The LQR controller achieved **stable balancing**: lean angle held within ±1° of target, angular velocity damped quickly, and controller output remained smooth and consistent. This was a significant improvement over PID.

All LQR source code and tuning files are in `src/balance/lqr/`.  
The live tuning dashboad is hosted by the ESP32 itself, after tuning it was taken off to take advantage of pure mathematical performance of the esp responsible for balancing the bicycle. 

---

## What's missing for next year

These are the open problems specifically for the balance system that the next team should focus on:

**Cornering stability**  
When the bike turns, the center of gravity needs to shift inward to correctly take a corner — just like a human cyclist leans into a bend. Currently there is no coupling between the balance system and the steering system. A proper implementation would have the balance controller intentionally tilt the bike during a turn, taking the apex smoothly rather than fighting to stay upright against the centrifugal force.

**Gain scheduled LQR**  
The current LQR uses fixed gains regardless of riding speed. At higher speeds the bike's dynamics change significantly. it becomes more self-stable and needs less aggressive correction. Wheel speed data from the Hall sensors (already available on the hardware) could be used to schedule different K values depending on velocity.

**Obstacle avoidance integration**  
Currently the bike has no autonomy and is fully remote-controlled. Future work could let the balance system lean the bike slightly sideways to dodge obstacles at an angle — similar to how a cyclist swerves, rather than relying only on steering or hard braking.

---

## Files & links

| What | Where |
|---|---|
| PID source code | `src/balance/pid/` |
| LQR source code | `src/balance/lqr/` |
| Tuning dashboard | `src/balance/dashboard/` |
| Final presentation (all calculations) | `docs/assets/FinalIB3.pdf` |

---

*IB3 — KU Leuven Campus Gent — 2025–2026*
