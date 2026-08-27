# Arm-Encoder-PCB

RS485 encoder interface board for accurate joint position feedback on the **UMD Mars Rover Team's** autonomous arm. The board polls a multi-drop bus of RS485 absolute encoders and relays live position and velocity data over CAN to the rest of the rover's control system, with an STM32 handling the protocol translation.

## Overview

Each joint on the autonomous arm is fitted with an RS485 absolute encoder (AMT21x / AMT212 series). This board's STM32 firmware manages a bus of these encoders, round-robin polling each one, validating the returned data, and tracking position/velocity per node. That data is exposed over CAN so the arm's main controller can request live feedback for any specific joint on demand.

## Node Addressing

Each encoder on the bus is assigned a unique node ID, sent directly as the command byte for a standard position read:


Up to a configurable max number of encoders can be registered at once, each tracked independently with its own node ID, resolution, poll period, last-known position, and online/offline status. Four encoders are loaded by default at startup, but encoders can be added or cleared at runtime, with duplicate node IDs rejected.

## Bus Polling

The system round-robins through registered encoders rather than polling all of them every cycle, respecting each encoder's individual poll period. Each controller cycle:
1. Checks whether the current encoder's poll interval has elapsed.
2. If due, sends that encoder's node-addressed command and reads back the 2-byte response.
3. Validates the response using the AMT21 parity check (two check bits derived from the data payload).
4. Decodes the position from the response, scaled to the encoder's configured resolution (up to 14-bit).
5. Computes velocity from the position delta between reads, unwrapping across the resolution's modulo range to handle wraparound at the 0/max boundary.
6. Advances to the next encoder for the following cycle.

Encoders that fail to respond or fail checksum validation are marked offline, with failed/successful read counters tracked per node — this lets the system distinguish a genuinely disconnected encoder from an occasional dropped packet.

## RS485 Transport

Direction control (TX/RX switching) is handled either automatically by the UART hardware or manually via a GPIO-driven DE pin, depending on configuration. Before every transmission, the receive buffer is flushed to avoid stale data corrupting the next read. Transmit completion is polled with a timeout rather than assumed, so a bus fault or non-responding node can't hang the polling loop indefinitely.

## CAN Integration

The board exposes encoder data over CAN request/response:

- **Request** — the controller specifies a target encoder's node ID.
- **Response** — the board replies with that node's ID, position (in 0.1° units), and velocity (in 0.01°/s units), scaled from raw encoder counts based on the encoder's resolution.

Requests are matched to a registered encoder by node ID; if the encoder isn't found or isn't currently online, no meaningful data is returned. This request/response model means the main controller can poll specific joints on demand rather than needing to parse a continuous broadcast stream.

## Reliability Notes

- Per-node online/offline tracking rather than a single global "encoder ok" flag
- Failed reads don't stall the polling loop — the round-robin simply moves to the next node
- Position wraparound is explicitly handled in velocity calculation, avoiding large erroneous velocity spikes when an encoder crosses its zero point
- Duplicate node IDs are rejected at registration, preventing bus address collisions from being configured accidentally
