# Arduino Telemetry

Arduino-based IMU telemetry system for model rockets. 

Phase 1: Build a basic Arduino project to learn basics of handling sensor data and developing on an MCU

Phase 2: Design and build a custom flight computer using accessible components and write flight-capable software.

Phase 3: Design and build a ground-control system to communicate with flight computer, handle launch sequence and startup, transmit data to and from the flight computer to a laptop.

Phase 4: Design and assemble the rocket airframe, mount the flight computer and Rotation Control System (RCS but not that kind ;P ), and test avionics and telemetry.

Phase 5: Build and test chute ejection system and determine required powder charge level.

Phase 6: Launch! 🚀

# Phase I - Research
## Temp Sensor PCB Design
### MK I
The MK I PCB was unfortunately unsuccessful, as I used the EasyEDA autoroute feature to try to solve track placement issues, however it did not take into account the possiblity of shorting the circuits, and neither did I, so my Arduino Nano blew a capacitor. Thankfully I have more, but this board was immediately decommissioned.

<img src="Images/Mk_I_F.png" width="40%" />
<img src="Images/Mk_I_B.png" width="40%" />

### MK II
The MK II PCB was carefully redesigned to improve upon the Mk I's failures by utilizing better component placement, built-in grounding pads, and hand-drawn track placement.

<img src="Images/Mk_II_F.png" width="40%" />
<img src="Images/Mk_II_B.png" width="40%" />

## Update:
Custom PCB worked after a slight modification to the electrical grounding system. Further research concluded that a "common ground" pin would have been more efficient and effective, than multiple pins.

<img src="Images/PCB-test.PNG" width="40%" />


# Phase II - Development
## Flight Computer
This is the assembled Mk.1 version of the flight computer, equipped with a barometer and IMU for tracking altitude, temperature, orientation, and velocity.

<img src="Images/flightCPU.PNG" width="40%" style="transform: rotate(270deg); margin-top: 65px; margin-bottom: 80px;" />
<img src="Images/groundCtrl.PNG" width="40%" style="transform: rotate(270deg); margin-top: 65px; margin-bottom: 80px;"/>
  

> UPDATE (MK. II)
>
> The flight computer has been upgraded to use the Adafruit Feather M0 with LoRa Radio @900Mhz, and the onboard IMU has been upgraded to the MPU 6050 with 6 DOF (and gyroscope!).

### MK.III
After working through various software iterations, the prototype board was complete and functional, but rather messy and utilized more patch wires than I preffered. Using the skills I learned from the PCB design in Phase 1, I began designing a new flight computer PCB, with the intent of using the same core components from the prototype board, allowing an easy transition of functionality, and requiring litte to no software logic changes. The Blep flight computer was developed, and serves as a major improvement in terms of efficiency, durability, and technological advancement. Having learned from my mistakes with the Phase 1 power management, I introduced a single common ground via on the PCB, as well as an entire "ground plane" within the layers of the PCB to ensure that all of the routes on the front and back of the board share a truly **common** ground.

<img src="Images/blep.png" width="40%" style="margin-top: 65px; margin-bottom: 80px;" />

# Phase III - Ground Control
## Mk. I
The AirLift Wi-Fi Co-Processor on both the Adalogger and Metro are able to successfully communicate with the Node-Red server, to send sensor data over UDP for collection and assimilation. The Node-Red dashboard is configured to not only build graphs in real-time, but also saves the data as it is received to a JSON file. 

<img src="Images/nodeRedDash.png" width="40%" />
<img src="Images/nodeRedGraph.png" width="40%" />  


## Mk. II
When the flight computer software is complete, the ground control system will be updated. Now that the onboard telemtry has shifted to use 900Mhz radio, the communication with the rocket will also be updated to use radio, and the data received by the ground control computer will be sent to a new server for analysis and logging. The new data analysis server utilizes Serial Studio instead of node-red, which is significantly more robust and efficient when dealing with high-speed data input, and doesn't suffer from the read/write operations to graphs, but still provides a JSON file as output for post-launch analysis. The primary MCU for the ground control computer will be a Teensy 3.2, connected to a LoRa Featherwing using the Teensy-Feather adapter board. 


# Phase IV - Rocket Design and Construction 
> ~~Currently doing research and teaching myself rocket mechanics to design and build a stable, efficient rocket.~~  
> UPDATE (9/2022): The rocket airframe is being designed simultaneous with Phase II, and rudimentary schematics will be included in this repo for future use. Old PCB schematics have been scrubbed from the project, as they are no longer relevant or necessary, and have been replaced by the Blep flight computer PCB files. Airframe construction will likely begin in the order planned.


# Phase V - Chute Ejection System
> Development still in progress...

# Phase VI - Launch!
> Development still in progress...
