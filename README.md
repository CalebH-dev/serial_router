# RS-232 Serial Router

## Overview
The **RS-232 Serial Router** is a microcontroller-based system designed to route visca packets between one camera controller and multiple cameras.It supports VISCA camera control, USB-serial interfacing (for debug and config), and flexible routing for master and slave devices.

It is intended to bridge the gap between daisy chain only visca controllers, and home-run only cameras.

## Features
- Bi-directional routing between multiple RS-232 devices
- VISCA camera power on/off control 
- USB terminal interface for configuration and debugging
- Non-volatile storage (NVS) for persistent device settings
- Customizable UART baud rates and channel addresses
- Real-time monitoring of received and transmitted packets
- LED status and activity indicators for visual feedback

## Hardware
Build your own. Needs level shifters like MAX3232.

## Software
- Written in **C++** for the RP2040 microcontroller  
- PIO UARTs and hardware UART 
- Circular packet queue for handling asynchronous communication
- Built-in configuration CLI for editing device settings via USB terminal

## Default Settings
| Parameter        | Default Value |
|------------------|---------------|
| CH1 ID           | 0x81          |
| CH2 ID           | 0x82          |
| CH1 Broadcast ID | 0x80          |
| CH2 Broadcast ID | 0x80          |
| Baud Rate        | 9600          |
| Response Header  | 0x90 / 0x91   |

Settings are stored in flash and verified on startup. If invalid, defaults are written automatically.

## Usage

- **Normal Operation:** The router automatically forwards packets between master and channels, monitors responses, and manages camera power.
- **Settings Config Mode:** Send at least one byte over USB to enter config mode. Available commands:
  - `list` – display current settings
  - `edit` – modify channel IDs, broadcast IDs, and baud rate
  - `default` – restore default settings
  - `update` – reboot into bootloader for firmware update (when bootsel button is hidden inside a case, etc.)
  - `exit` – return to normal operation

- **Camera Power Control:** Use the master power switch to toggle all connected cameras on/off. System assums the operation succedes to avoid getting stuck.

## Contribution

- This project is almost complete for my purposes, but contribution is more than welcome.
- Ensure any changes maintain compatibility with as many existing protocol variations and as possible.

