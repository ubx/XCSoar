// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "System.hpp"
#include "system/FileUtil.hpp"
#include "system/PathName.hpp"
#include "system/Process.hpp"
#include "system/Sleep.h"
#include "util/StaticString.hxx"

bool
IsColibriWifiOn()
{
  StaticString<64> path;
  path.Format("/sys/class/net/%s", "wlan0");
  return Directory::Exists(Path{path});
}

bool
IsColibriWifiAutoOn()
{
  return false;
}

bool
ColibriWifiOn()
{
  Run("/sbin/ifconfig", "wlan0", "up");
  return true;
}

bool
ColibriWifiOff()
{
  Run("/sbin/ifconfig", "wlan0", "down");
  return true;
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
  Run("/opt/XCSoar/bin/xcsoar", "-datapath=/media/mmcblk0p2/xcsoar-data/", mode);
}

bool
IsColibriCustomKernel()
{
  return false;
}

bool
ColibriPowerOff()
{
  return Run("/sbin/poweroff");
}

bool
ColibriReboot()
{
  return Run("/sbin/reboot");
}
