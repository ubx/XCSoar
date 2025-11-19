// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include <linux/can.h>
#include <string>

/*
 * Parse a single SLCAN line into a Linux can_frame.
 *
 * Accepted formats (classic SLCAN):
 *  - 't' sID(3 hex) DLC(1 hex) [data bytes 0..8 as 2 hex chars each]
 *  - 'T' eID(8 hex) DLC(1 hex) [data bytes ...]
 *  - 'r'/'R' same as above but RTR (no payload)
 *
 * Notes:
 *  - Standard IDs must be <= 0x7FF; extended IDs <= 0x1FFFFFFF.
 *  - DLC range: 0..8 (classic CAN).
 *  - Trailing characters are not allowed, except optional CR/LF terminators.
 *  - The input string may include optional trailing '\r'/'\n'.
 *
 * Output mapping to struct can_frame:
 *  - f.can_id contains the CAN identifier; flags are applied as needed:
 *      - CAN_EFF_FLAG set for extended frames, identifier masked to 29 bits
 *      - CAN_RTR_FLAG set for RTR frames
 *  - f.can_dlc set to parsed DLC (0..8)
 *  - f.data[0..can_dlc-1] filled for data frames; undefined for RTR
 */
bool parse(const std::string &line, struct can_frame &f);