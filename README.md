# CanSat2025_FSW


 When pulling the main branch, run the "Device Configuration Tool Code Generation." The button is in the top bar and has a yellow gear and prevents build errors.

Todo: 
1. Setup I2C, SPI, and UART connection to related sensors
2. Setup GPIO pins for motors
3. Transmission of telemetry packet struct
4. Aperiodic thread, ie interrupt for incoming transmission
5. Setup interthread communication, either via global flag, queue, or other method 
6. Verify specific pins for each system as laid out on PCB
