// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "System.hpp"

bool
IsColibriWifiOn()
{
  return false;
}

bool
IsColibriWifiAutoOn()
{
  return false;
}

bool
ColibriWifiOn()
{
  return false;
}

bool
ColibriWifiOff()
{
  return false;
}

bool
SetColibriWifiAutoOn(bool enabled)
{
  (void)enabled;
  return false;
}

void
ApplyColibriWifiAutoOn()
{
}

void
ColibriRunTelnetd()
{
}

void
ColibriRunFtpd()
{
}

void
ColibriRunXCSoar(const char *mode)
{
  (void)mode;
}

bool
IsColibriCustomKernel()
{
  return false;
}

bool
ColibriPowerOff()
{
  return false;
}

bool
ColibriReboot()
{
  return false;
}
