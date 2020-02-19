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

#include "Device/Driver/CANaerospace.hpp"
#include "Device/Driver.hpp"
#include "NMEA/Info.hpp"

#include <linux/can.h>

#include <iostream> // TODO: Remove this, its just for debugging
#include <iomanip> // TODO: Remove this, its just for debugging
#include <Device/Driver/CANaerospace/marshal.h>
#include <map>
#include <Time/RoughTime.hpp>
#include <Time/LocalTime.hpp>
#include <Device/Port/CANPort.hpp>

class CANaerospaceDevice : public AbstractDevice {
//  Port &port;

  public:
    CANaerospaceDevice(Port &_port)  {
    }
    bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;
};

std::map<int, double > canId2clock;
auto last_fix = GeoPoint::Invalid();

static bool
SouldSend(int can_id, double clock) {
//    if (canId2clock[can_id] + 1.0 <= clock)  {
//        canId2clock[can_id] = clock;
//        return true;
//    }
//    return false;
   return true;
}

static Device *
CANaerospaceCreateOnPort(const DeviceConfig &config, Port &com_port)
{
    return new CANaerospaceDevice(com_port);
}

bool
CANaerospaceDevice::DataReceived(const void *data, size_t length,
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
                if (last_fix.Check()) {
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
                info.time = TimeLocal(info.date_time_utc.GetSecondOfDay(), RoughTimeDelta()); // todo -- verify !!
                info.time_available.Update(info.clock);
                return true;
            }
            break;

        case HEADING_ANGLE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                float value = phost->container.FLOAT;
                if (value < 0.0) {
                    value += 360.0;
                }
                info.heading = Angle::Degrees(value);
                info.heading_available.Update(info.clock);
                return true;
            }
            break;

        case GPS_TRUE_TRACK:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.track = Angle::Degrees(phost->container.FLOAT);
                info.track_available.Update(info.clock);
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

        case AIRMASS_SPEED_VERTICAL:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideNettoVario(phost->container.FLOAT);
                return true;
            }
            break;

        case STATIC_PRESSURE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideStaticPressure(AtmosphericPressure::HectoPascal(phost->container.FLOAT));
                return true;
            }
            break;

        case STANDARD_ALTITUDE:
            if (canasNetworkToHost(phost, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideBaroAltitudeTrue(phost->container.FLOAT);
                return true;
            }
            break;


        default:
            std::cout << "not implemented can_id: "  << canFrame->can_id  << std::endl;
            break;
    }
    return false;
}

const struct DeviceRegister can_aerospace_driver = {
        _T("CANaerospace"),
        _T("CANaerospace"),
  DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA, // TODO: Put the right flags
  CANaerospaceCreateOnPort,
};
