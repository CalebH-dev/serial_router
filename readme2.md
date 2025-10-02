# RS-232 Serial Router

## Overview
The **RS-232 Serial Router** is a microcontroller-based system designed to manage, route, and monitor multiple RS-232 serial communication channels. It supports VISCA camera control, USB-serial interfacing, and flexible routing for master and slave devices.

This project is ideal for setups requiring reliable communication between multiple serial devices while maintaining configurable addresses, baud rates, and broadcast channels.

## Features
- Bi-directional routing between multiple RS-232 devices
- VISCA camera power control (`CAM_ON`, `CAM_OFF`, `CAM_INQ`)
- USB terminal interface for configuration and debugging
- Non-volatile storage (NVS) for persistent device settings
- Customizable UART baud rates and channel addresses
- PIO-based UART handling for precise timing and signal control
- Real-time monitoring of received and transmitted packets
- PWM-driven LED indicator for visual status feedback

## Hardware
- **Microcontroller:** [Raspberry Pi Pico or compatible RP2040 board]  
- **RS-232 Level Shifter:** MAX3232 (3.3V logic to RS-232 voltage)  
- **Connectors:** DB-9 (Amphenol CS FCI series recommended)  
- **GPIO Usage:**  
  - TX/RX pins for UART0 and UART1  
  - PWM pins for status LEDs  
  - Switch pins for master power and NVS mode  
  - Error indicator pin  

> Ensure correct pin assignments. Pins 23-25 are unused.

## Software
- Written in **C++** for the RP2040 microcontroller  
- Uses Raspberry Pi Pico SDK libraries (`pico/stdlib.h`, `hardware/uart.h`, `hardware/pio.h`, `hardware/pwm.h`)  
- PIO programs manage UART receive and transmit with precise timing  
- Circular packet queue for handling asynchronous communication
- Built-in configuration CLI for editing device settings via USB terminal

## Default Settings
| Parameter        | Default Value |
|-----------------|---------------|
| CH1 ID           | 0x81          |
| CH2 ID           | 0x82          |
| CH1 Broadcast ID | 0x80          |
| CH2 Broadcast ID | 0x80          |
| Baud Rate        | 9600          |
| Response Header  | 0x90 / 0x91   |

Settings are stored in flash and verified on startup. If invalid, defaults are written automatically.

## Setup

1. Connect the RS-232 devices using DB-9 connectors and level shifters.
2. Flash the firmware onto the RP2040 board using your preferred method.
3. Connect the board via USB for configuration and monitoring.
4. Ensure proper wiring for master power switch and error indicator pins.
5. Optional: Configure UART addresses, broadcast IDs, and baud rates via the built-in CLI.

## Usage

- **Normal Operation:** The router automatically forwards packets between master and channels, monitors responses, and manages camera power.
- **NVS/Settings Mode:** Hold the NVS switch to enter configuration mode on boot or press during operation. Available commands:
  - `list` – display current settings
  - `edit` – modify channel IDs, broadcast IDs, and baud rate
  - `default` – restore default settings
  - `update` – reboot into bootloader for firmware update
  - `exit` – return to normal operation

- **Camera Power Control:** Use the master power switch to toggle all connected cameras on/off. System performs inquiry to determine current state.

## Troubleshooting

- **Signal Issues:** Verify RX/TX wiring; MAX3232 should handle voltage conversion.
- **Overflows:** Buffers have a fixed size (32 bytes); ensure master devices respect packet limits.
- **Invalid Commands:** Devices must use valid addresses (0x80–0x88 for master/broadcast, 0x81/0x82 for channels).
- **Debugging:** Connect via USB terminal (e.g., `minicom` or `GTKTerm`) to monitor received and transmitted packets in real-time.

## Contribution

- Contributions and improvements are welcome via pull requests.
- Ensure any changes maintain backward compatibility with existing protocol addresses and commands.

## License
[Insert license here]

