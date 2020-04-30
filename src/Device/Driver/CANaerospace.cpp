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
#include <Device/Driver/FLARM/flarmPropagated.hpp>

class CANaerospaceDevice : public AbstractDevice {
//  Port &port;

public:
    CANaerospaceDevice(Port &_port) {
    }

    bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;
};

std::map<int, double> canId2clock;
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
CANaerospaceCreateOnPort(const DeviceConfig &config, Port &com_port) {
    return new CANaerospaceDevice(com_port);
}

bool
CANaerospaceDevice::DataReceived(const void *data, size_t length,
                                 NMEAInfo &info) {

    const auto *canFrame = (const can_frame *) data;   // Cast the adress to a can frame
    auto *canData = canFrame->data + 4;
    auto *canasMessage = new(CanasMessage);
    auto canasData = &canasMessage->data;


    const auto *canasMessage2 = (const CanasMessage *) canFrame->data;
    canasMessage->message_code = canasMessage2->message_code;
    canasMessage->service_code = canasMessage2->service_code;

    assert(data != nullptr);
    assert(length > 0);
    assert(canData != nullptr);

    info.alive.Update(info.clock);
    if (!SouldSend(canFrame->can_id, info.clock) && canFrame->can_id != GPS_AIRCRAFT_LATITUDE) {
        return false;
    }

    switch (canFrame->can_id) {
        case GPS_AIRCRAFT_LATITUDE:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.latitude = Angle::Degrees(canasData->container.LONG / 1E7);
            }
            break;

        case GPS_AIRCRAFT_LONGITUDE:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.longitude = Angle::Degrees(canasData->container.LONG / 1E7);
                if (last_fix.Check()) {
                    info.location = last_fix;
                    info.location_available.Update(info.clock);
                    return true;
                }
            }
            break;

        case GPS_AIRCRAFT_HEIGHTABOVE_ELLIPSOID:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.gps_altitude = canasData->container.FLOAT;
                info.gps_altitude_available.Update(info.clock);
                return true;
            }
            break;

        case UTC:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_CHAR4) > 0) {
                info.date_time_utc.hour = canasData->container.CHAR4[0];
                info.date_time_utc.minute = canasData->container.CHAR4[1];
                info.date_time_utc.second = canasData->container.CHAR4[2];
                info.time = TimeLocal(info.date_time_utc.GetSecondOfDay(), RoughTimeDelta()); // todo -- verify !!
                info.time_available.Update(info.clock);
                return true;
            }
            break;

        case HEADING_ANGLE:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                float value = canasData->container.FLOAT;
                if (value < 0.0) {
                    value += 360.0;
                }
                info.heading = Angle::Degrees(value);
                info.heading_available.Update(info.clock);
                return true;
            }
            break;

        case GPS_TRUE_TRACK:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.track = Angle::Degrees(canasData->container.FLOAT);
                info.track_available.Update(info.clock);
                return true;
            }
            break;

        case INDICATED_AIRSPEED:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.indicated_airspeed = canasData->container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case TRUE_AIRSPEED:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.true_airspeed = canasData->container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case GPS_GROUND_SPEED:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ground_speed = canasData->container.FLOAT;
                info.ground_speed_available.Update(info.clock);
                return true;
            }
            break;

        case AIRMASS_SPEED_VERTICAL:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideNettoVario(canasData->container.FLOAT);
                return true;
            }
            break;

        case STATIC_PRESSURE:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideStaticPressure(AtmosphericPressure::HectoPascal(canasData->container.FLOAT));
                return true;
            }
            break;

        case STANDARD_ALTITUDE:
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideBaroAltitudeTrue(canasData->container.FLOAT);
                return true;
            }
            break;

        case FLARM_STATE_ID:  // Flarm messages: PFLAU
            // PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>, <RelativeVertical>,<RelativeDistance>
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_CHAR4) > 0) {

                static FlarmState S;
                static FlarmMostImportantObjectData O;

                if (canasFlarmStatePropagated(canasMessage, info.gps_altitude, &O, &S)) {
                    info.flarm.status.available.Update(info.clock);
                    info.flarm.status.rx = S.RxDevicesCount;
                    info.flarm.status.tx = S.TxState;
                    info.flarm.status.gps = (FlarmStatus::GPSStatus) S.GpsState;
                    info.flarm.status.alarm_level = (FlarmTraffic::AlarmType) O.AlarmLevel; // TODO -- correct ???
                    return true;
                }
            }
            break;

        case FLARM_OBJECT_AL3_ID: // Flarm messages: PFLAA
        case FLARM_OBJECT_AL2_ID:
        case FLARM_OBJECT_AL1_ID:
        case FLARM_OBJECT_AL0_ID:
            // PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<ID-Type>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<Type>
            if (canasNetworkToHost(canasData, canData, 4, CANAS_DATATYPE_UCHAR4) > 0) {

                static FlarmObjectData E;

                if (canasFlarmObjectPropagated(canasMessage, info.heading.Degrees(), canFrame->can_id, &E)) {
                    FlarmTraffic traffic;
                    traffic.alarm_level = (FlarmTraffic::AlarmType) E.AlarmLevel;
                    traffic.relative_north = E.RelNorth;
                    traffic.relative_east = E.RelEast;
                    traffic.relative_altitude = E.RelHorizontal;
                    traffic.id.Set(E.ID);
                    info.flarm.traffic.FindTraffic(traffic.id);

                    FlarmTraffic *flarm_slot = info.flarm.traffic.FindTraffic(traffic.id);
                    if (flarm_slot == nullptr) {
                        flarm_slot = info.flarm.traffic.AllocateTraffic();
                        if (flarm_slot == nullptr) {
                            // no more slots available
                            return false;
                        }

                        flarm_slot->Clear();
                        flarm_slot->id = traffic.id;

                        info.flarm.traffic.new_traffic.Update(info.clock);
                    }
                    // set time of fix to current time
                    flarm_slot->valid.Update(info.clock);
                    flarm_slot->Update(traffic);
                    assert(traffic.id.IsDefined());
                    return true;
                }
            }
            break;


        default:
            std::cout << "not implemented can_id: " << canFrame->can_id << std::endl;
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
