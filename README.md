# Robot_Platform_Firmware

This repo contains the CUBE MX configuration file and the source code to control a holonomic robotic platform using ST MicroController.

It depends on the particular motors, drivers, and high-level control used, but you could find some useful informations:

In the MX config file:

- How to setup TIMERS in Encoder Mode.
- How to setup serial communication
- How to setup LWIP.
- How to setup FreeRTOS.

In the source code:

- How to handle UDP communication.
- How to handle serial communication.
- How to handle FreeRTOS tasks logic.
- Most importantly, how to make LEDs blink.
