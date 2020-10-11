# Arduino Telemetry

Arduino-based IMU telemetry system for model rockets. 

Phase 1: Build a weather station with custom PCB to learn basics of PCB design and sensor data

Phase 2: Using knowledge from Phase 1, design custom rocket avionics board using off-the-shelf components and test in rocket.

Phase 3: Build ground-control system to communicate with avionics on board

Phase 4: Build rocket

Phase 5: Test rocket design and launch! 🚀

# Phase I
## PCB Schematics
### MK I
The MK I PCB was unfortunately unsuccessful, as I used the EasyEDA autoroute feature to try to solve track placement issues, however it did not take into account the possiblity of shorting the circuits, and neither did I, so my Arduino Nano blew a capacitor. Thankfully I have more, but this board was immediately decommissioned.

<img src="Images/Mk_I_F.png" width="60%" />
<img src="Images/Mk_I_B.png" width="60%" />

### MK II
The MK II PCB was carefully redesigned to improve upon the Mk I's failures by utilizing better component placement, built-in grounding pads, and hand-drawn track placement. (Blue also seemed like it might look nicer.) 

<img src="Images/Mk_II_F.png" width="60%" />
<img src="Images/Mk_II_B.png" width="60%" />
