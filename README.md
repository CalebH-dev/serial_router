# RS-232 Serial Router

## Overview
RS-232 Visca Serial Router is a hardware and software project designed to manage and route Visca packets over RS-232 serial communications. It supports 2 device-side serial connections and allows for flexible routing between devices with fine-grained configureation.

The project uses a combination of microcontroller firmware and standard RS-232 hardware, including MAX3232 level shifters and DB-9 connectors.

## Features
- Multi-port RS-232 routing
- VISCA camera control support
- USB-to-Serial interface support (Possable addition)
- Signal quality monitoring and handling

## Hardware
- Microcontroller: Raspberry Pi RP2030 Module
- RS-232 Level Shifter: MAX3232
- Connectors: DB-9 Male/Female (controller input and pass-through), 2x RJ45 (device-side connections)

## Setup
1. Connect cammeras to RJ45 ports and controller to female db-9 "input"
2. Conect over USB serial to figure settings as needed:
   - Baud rate: [Default: 9600]
   - ID to port mapping [Default: 1 --> port 1, 2 --> port 2]

2. [Set Broadcast ]
