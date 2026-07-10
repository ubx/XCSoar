// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "net/wifi/WifiBackend.hpp"

/**
 * CreatePlatformWifiBackend() returns the Colibri-specific UniqueWifiBackend
 * factory result backed by wpa_supplicant.
 */
UniqueWifiBackend
CreatePlatformWifiBackend();
