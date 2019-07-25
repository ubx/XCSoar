/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Device/Driver/VegaCAN.hpp"
#include "Device/Driver.hpp"
#include "NMEA/Info.hpp"

#include <linux/can.h>

#include <iostream> // TODO: Remove this, its just for debugging
#include <iomanip> // TODO: Remove this, its just for debugging
#include <Device/Driver/VegaCAN/marshal.h>
#include <map>

class VegaCANDevice : public AbstractDevice {
  Port &port;

  public:
    VegaCANDevice(Port &_port):port(_port) {}

    bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;

};

GeoPoint last_fix = GeoPoint::Invalid();
std::map<int, double > canId2clock;

static bool
SouldSend(int can_id, double clock) {
    if (canId2clock[can_id] + 1.0 <= clock)  {
        canId2clock[can_id] = clock;
        return true;
    }
    return false;
}

static Device *
VegaCANCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new VegaCANDevice(com_port);
}

bool
VegaCANDevice::DataReceived(const void *data, size_t length,
                           NMEAInfo &info) {

    const can_frame *canFrame = (const can_frame *) data;   // Cast the adress to a can frame
    const auto *canData = canFrame->data + 4;
    auto *phost = new(CanasMessageData);

    assert(data != nullptr);
    assert(length > 0);
    assert(canData != nullptr);

    info.alive.Update(info.clock);
    if (!SouldSend(canFrame->can_id, info.clock) && canFrame->can_id != GPS_AIRCRAFT_LATITUDE) {
        return false;
    }

    switch (canFrame->can_id) {
        case GPS_AIRCRAFT_LATITUDE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.latitude = Angle::Degrees(phost->container.LONG  / 1E7);
            }
            break;

        case GPS_AIRCRAFT_LONGITUDE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.longitude = Angle::Degrees(phost->container.LONG  / 1E7 );
                if (last_fix.IsValid()) {
                    info.location = last_fix;
                    info.location_available.Update(info.clock);
                    return true;
                }
            }
            break;

        case GPS_AIRCRAFT_HEIGHTABOVE_ELLIPSOID:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.gps_altitude = phost->container.FLOAT;
                info.gps_altitude_available.Update(info.clock);
                return true;
            }
            break;

        case UTC:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_CHAR4) > 0) {
                info.date_time_utc.hour = phost->container.CHAR4[0];
                info.date_time_utc.minute = phost->container.CHAR4[1];
                info.date_time_utc.second = phost->container.CHAR4[2];
                info.time_available.Update(info.clock);
                return true;
            }
            break;

        case HEADING_ANGLE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.heading = Angle::Native(phost->container.FLOAT);
                info.heading_available.Update(info.clock);
                return true;
            }
            break;

        case INDICATED_AIRSPEED:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.indicated_airspeed = phost->container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case TRUE_AIRSPEED:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.true_airspeed = phost->container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case GPS_GROUND_SPEED:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ground_speed = phost->container.FLOAT;
                info.ground_speed_available.Update(info.clock);
                return true;
            }
            break;

        case AIRMASS_SPEED_VERTICAL:  // todo -- ???
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.netto_vario = phost->container.FLOAT;
                info.netto_vario_available.Update(info.clock);
                return true;
            }
            break;

        case STATIC_PRESSURE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.static_pressure = AtmosphericPressure::HectoPascal(phost->container.FLOAT);
                info.static_pressure_available.Update(info.clock);
                return true;
            }
            break;

        case STANDARD_ALTITUDE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.baro_altitude = phost->container.FLOAT;
                info.baro_altitude_available.Update(info.clock);
                return true;
            }
            break;


        default:
            std::cout << "not implemented can_id: "  << canFrame->can_id  << std::endl;
            break;
    }
    return false;
}

const struct DeviceRegister vega_can_driver = {
  _T("VegaCAN"),
  _T("VegaCAN"),
  DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA, // TODO: Put the right flags
  VegaCANCreateOnPort,
};
