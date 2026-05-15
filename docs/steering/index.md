# Steering System — IB3 Self-Driving Bicycle 2025–2026

**Team:** Group 1 (Robin, Quinten, Maxime, Lars, Noah, Lee) & Group 2 (Milan, Kayen, Thibaut, Toon, Simon V, Simon VK)  
**Note:** Both groups took a different route on the steering system. We will try to separate them out as much as possible.

For full slides, and detailed comparisons, refer to the [final presentation (PDF)](../assets/FinalIB3.pdf).

---

## Contents

1. [Problem statement](#problem-statement)
2. [Research: dimensioning a motor](#research-dimensioning-a-motor)
3. [Plan A — Belt Drive (group 2)](#plan-a-belt-drive)
4. [Plan B — Chain Drive (group 2)](#plan-b-chain-drive)
5. [Plan Direct Drive (group 1)](#plan-direct-drive)

---

## Problem statement

The goal is a bicycle that stays upright and moves without any physical human assistance. The steering system makes steering the bike possible.

---

## Research: dimensioning a motor

A portable luggage scale was used to pull on the steering wheel of the bike, to find the maximum pull strength required to move the steering wheel. Expirementally was found that a force of $F = 19,62 N (= 2 kg)$ at a distance of $30 cm$ was required to move the steering wheel. Using the moment's law of static mechanics:

$$M_{N}= F \cdot r = 19,6 N \cdot 0,3 m= 5,88 Nm \approx 60 kg \cdot cm$$

There's a handy [convertor tool online](https://lucidar.me/en/unit-converter/convert-torque-in-n-m-to-kg-cm/) to help convert Nm to kg cm. A lot of (servo) motors online will list their torque as kg cm. 

Group 1 took a 300% safety margin into account and used a moment's force of:

$$M_{N}= 3 \cdot F \cdot r = 3\cdot  19,6 N \cdot 0,3 m= 5,88 Nm \approx 180 kg \cdot cm$$

---

## Plan A Belt Drive (group 2)

Group 2's first approach was using a belt drive. They tried using a 6mm wide 2GT type belt, known for their use in 3d printers.
The belt was bought on ALiExpress, as a package with 120 teeth gear (bore 25mm, same as the steering post diameter). This build was accompanied by a 3d printed gear (melted into the metal accessory arm that came with the servo).
This did not end up working. Reason: the belt started slipping while steering, making it impossible to reliably steer.

---

## Plan B Chain Drive (group 2)

A quick and dirty solution to the slipping belt was made (literally hot glued) on top of the old belt gears: bike gears with a chain.
This chain performed much better than the belt, because it couldn't slip as easily.

**Why this works:**
- chain does not dynamically flex like a belt
- near 1:1 gear ratio makes setting steering angle very easy: servo angle = steering angle

**Downsides to be aware of:**
- there is some play (backlash) in the chain. This will probably end up in vibrations on higher speeds 
- installing and adjusting is a cumbersome process: screws involved.


For hardware details (Perfboard layout, IMU wiring, ESC connections, bill of materials), see [Hardware Implementation](hardware_implementation.md).

---



## Plan Direct Drive (group 1)
Group 1 put all their eggs, hopes and dreams on 1 solution: direct drive using selfmade gears. They tried 3d printing them, but that ended up not being strong enough. They tried lasercutting, but again, they were defeated by the much too strong moment's force of their chunky servo motor. At last, they finally used a very thick lasercut gear in combination with a 3d printed gear to finally overcome the crushing power of their own (badly chosen) motor. 
### How it works

### Tuning parameters


### Final settings (Group 2)



### Result



---


*IB3 — KU Leuven Campus Gent — 2025–2026*
