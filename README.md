# TI CC3220S XL LaunchPad Projects
## Morse Code Embedded System with State Machine
This project programs the launchpad to communicate in morse code with the GPIO interface, and changes messages when a button is pressed. The message does not change until the current message is complete. The red LED for 500ms indicates dot, while the green LED for 1500ms indicates dash. The embedded system utilizes the GPIO interface to achieve the morse code functionality.
## Thermostat Embedded System with State Machine
The aim of the project was to make a thermostat from the TI CC3220S XL Launchpad, using the GPIO, I2C, and UART2 interfaces along with timers and interrupts. The I2C interface reads the room's temperature via the launchpad's temperature sensor, then the UART2 interface displays the temperature to the console. The GPIO interface lights up an LED when the temperature is lower than the setpoint, indicating that the heater is on.
