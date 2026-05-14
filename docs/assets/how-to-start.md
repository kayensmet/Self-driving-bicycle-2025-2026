
# Before You Start: Startup & Safety Guide

Before operating the self-driving bicycle, please follow this exact sequence to ensure all systems are powered correctly and safely.

## 1. Powering the Hardware

*   **Main Battery (Drive System):** First, check if the main battery is on. You can find it under the white wingflaps. Look for the **red button**, which indicates the on/off state. This provides power to the wheel and the drive board (with the ESP).
*   **LiPo Battery (Balancing System):** To power the balancing system, locate the LiPo battery mounted under the wooden white wingflaps. Take its **yellow connector** and plug it into the matching yellow connector located around the seat pole. 
    *   *Note:* This supplies power to both propellers and, via a buck converter, powers the steering PCB. For more details on how the ESCs, propellers, IMU, and the balancing board interact, see the [Balancing](src/balance) documentation.

Side note: Make sure the batteries are charged as well ;) 
## 2. Controller Setup

The remote controller does not run on internal batteries. **You must plug a power bank into the controller** to power it on. 

Before activating the bike, ensure your controller is in the following safe baseline state:
*   **Left Sliders (Forward & Brake):** Push all the way **DOWN**.
*   **Right Slider (Steer):** Set to the **MIDDLE**.
*   **Top Right Buttons (3 switches):** Set all three switches to the **MIDDLE** position.
    *   *Left Button:* Buzzer (indicates that the bike is on).
    *   *Middle Button:* Balancer activation (see next step).
    *   *Right Button:* Originally intended for the safety PCB/relay (currently not implemented as the relay did not function properly).

## 3. Activation & Safety Rules

*   **Environment:** Ensure the bike is in a safe, open, and controlled area before turning anything on.
*   **Starting Position:** The bike is capable of starting up by itself directly from its stand. **Do not** attempt to start the balancing system from an angle lower than the stand's natural resting point. This has not been tested and is highly discouraged.
*   **Activation:** Once all checks are complete and the area is clear, push the **middle button** on the top right of the controller **UP** to activate the balancing system!
