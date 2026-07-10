// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "PlatformWifiBackend.hpp"
#include "WPASupplicantBackend.hpp"

#include <memory>

UniqueWifiBackend
CreatePlatformWifiBackend()
{
  return std::make_unique<WPASupplicantBackend>("wlan0");
}
