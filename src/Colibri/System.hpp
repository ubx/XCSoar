// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

bool
IsColibriWifiOn();

bool
IsColibriWifiAutoOn();

bool
ColibriWifiOn();

bool
ColibriWifiOff();

bool
SetColibriWifiAutoOn(bool enabled);

void
ApplyColibriWifiAutoOn();

void
ColibriRunTelnetd();

void
ColibriRunFtpd();

void
ColibriRunXCSoar(const char *mode);

bool
IsColibriCustomKernel();

bool
ColibriPowerOff();

bool
ColibriReboot();
