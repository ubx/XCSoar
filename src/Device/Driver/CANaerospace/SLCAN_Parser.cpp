// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SLCAN_Parser.hpp"
#include <cstdint>

namespace {
/* ---------------------------------------------------------
 * Convert a hex digit ('0'-'9','A'-'F','a'-'f') into a nibble value
 * Returns -1 on error
 * --------------------------------------------------------- */
int
hex_value(char c)
{
  if (std::isdigit(c)) return c - '0';
  c = std::tolower(static_cast<unsigned char>(c));
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* ---------------------------------------------------------
 * Read N hex chars and convert them into a 32-bit unsigned integer
 * Returns true on success, false on error (invalid digit)
 * --------------------------------------------------------- */
bool
read_hex_n(const char *p, int digits, uint32_t &out)
{
  uint32_t v = 0;
  for (int i = 0; i < digits; ++i) {
    int h = hex_value(p[i]);
    if (h < 0) return false;
    v = (v << 4) | h;
  }
  out = v;
  return true;
}
} // namespace

/* ---------------------------------------------------------
 * Parse an SLCAN line into a CAN frame
 * --------------------------------------------------------- */
bool
parse(const std::string &line, struct can_frame &f)
{
  const char *p = line.c_str();
  size_t len = line.length();
  if (len < 4) return false;

  char type = p[0];

  // Determine frame type
  const bool extended = (type == 'T' || type == 'R');
  const bool rtr = (type == 'r' || type == 'R');

  if (type != 't' && type != 'T' && type != 'r' && type != 'R') return false;

  size_t pos = 1;

  // ID length depends on extended flag
  int id_len = extended ? 8 : 3;

  if (len < pos + static_cast<size_t>(id_len) + 1) return false;

  uint32_t id_val = 0;
  if (!read_hex_n(p + pos, id_len, id_val)) return false;

  uint32_t parsed_id = id_val;
  pos += id_len;

  // DLC
  int dlc_val = hex_value(p[pos]);
  if (dlc_val < 0 || dlc_val > 8) return false;

  uint8_t dlc = (uint8_t)dlc_val;
  pos++;

  // Enforce legal CAN identifier ranges
  if (!extended && parsed_id > 0x7FFu) return false;
  if (extended && parsed_id > 0x1FFFFFFFu) return false;

  // Initialize can_frame fields
  f.can_dlc = dlc;
  f.can_id = 0;
  if (extended) {
    f.can_id = (parsed_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    f.can_id = (parsed_id & CAN_SFF_MASK);
  }
  if (rtr) f.can_id |= CAN_RTR_FLAG;

  // RTR frames have no payload
  if (rtr) {
    // Allow only optional CR/LF after DLC
    while (pos < len && (p[pos] == '\r' || p[pos] == '\n'))
      ++pos;
    return pos == len;
  }

  if (len < pos + static_cast<size_t>(dlc) * 2) return false;

  // Parse payload bytes
  for (int i = 0; i < dlc; i++) {
    uint32_t byte_val = 0;
    if (!read_hex_n(p + pos, 2, byte_val)) return false;
    f.data[i] = static_cast<uint8_t>(byte_val & 0xFFu);
    pos += 2;
  }

  // Allow only optional CR/LF at the end
  while (pos < len && (p[pos] == '\r' || p[pos] == '\n'))
    ++pos;

  return pos == len;
}