## File Descriptions

- **Process_sensor_data.c** → Handles logging of sensor data and system status.
- **inFlash.c** → Manages internal flash storage, including read/write operations.
- **mpu9250.c** → Interfaces with the MPU9250 IMU sensor for motion tracking via I2C communication.
- **nandflash_controller.c** → Controls NAND flash memory operations like reading, writing, and erasing by GPIO pins.
- **nand_hal_gpio.c** → Configures GPIO pins for NAND memory access.
- **ndelay.c** → Implements time delay functions for precise timing.
- **messageParser_gps.c** → Parses NMEA GPS data received via UART.
- **command_to_pc.c** → Manages serial communication between the MCU and a PC.
- **transfer_data_to_esp32.c** → Handles Wi-Fi connectivity and data transmission.
