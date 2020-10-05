# Arduino Telemetry

This code outlines how to read and display the serial data from the Adafruit BME280 Temperature Sensor on an Arduino Nano, and display data on an I2C LCD. (In this particular case, a 16x2 LCD)


This is a prototype for a future project which will incorporate this weather station alongside an accelerometer to work as the telemetry for a model rocket. 

# PCB Schematics
## MK I
The MK I PCB was unfortunately unsuccessful, as I used the EasyEDA autoroute feature to try to solve track placement issues, however it did not take into account the possiblity of shorting the circuits, and neither did I, so my Arduino Nano blew a capacitor. Thankfully I have more, but this board was immediately decommissioned.
![](Images/Mk_I_F.png)
![](Images/Mk_I_B.png)

## MK II
The MK II PCB was carefully redesigned to improve upon the Mk I's failures by utilizing better component placement, built-in grounding pads, and hand-drawn track placement. (Blue also seemed like it might look nicer.) 
> This board is currently in production, and has not shipped yet, will update status and results once board arrives

![](Images/Mk_II_F.png)
![](Images/Mk_II_B.png)
