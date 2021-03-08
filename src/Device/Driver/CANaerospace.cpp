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
#include <time/RoughTime.hpp>
#include <time/LocalTime.hpp>
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

static FlarmState flarmState;
static FlarmMostImportantObjectData objectData;
static FlarmObjectData flarmObjectData;

static Device *
CANaerospaceCreateOnPort(const DeviceConfig &config, Port &com_port) {
    return new CANaerospaceDevice(com_port);
}

bool
CANaerospaceDevice::DataReceived(const void *data, size_t length,
                                 NMEAInfo &info) {

    assert(data != nullptr);
    assert(length > 0);

    const can_frame *canFrame = (const can_frame *) data;   // Cast the adress to a can frame
    const auto *canData = canFrame->data + 4;
    const CanasMessage *cm = (const CanasMessage *) canFrame->data;

    static CanasMessage canasMessage;
    canasMessage.message_code = cm->message_code;
    canasMessage.service_code = cm->service_code;

    info.alive.Update(info.clock);

    switch (canFrame->can_id) {
        case GPS_AIRCRAFT_LATITUDE:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.latitude = Angle::Degrees(canasMessage.data.container.LONG / 1E7);
            }
            break;

        case GPS_AIRCRAFT_LONGITUDE:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.longitude = Angle::Degrees(canasMessage.data.container.LONG / 1E7);
                if (last_fix.Check()) {
                    info.location = last_fix;
                    info.location_available.Update(info.clock);
                    return true;
                }
            }
            break;

        case GPS_AIRCRAFT_HEIGHTABOVE_ELLIPSOID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.gps_altitude = canasMessage.data.container.FLOAT;
                info.gps_altitude_available.Update(info.clock);
                return true;
            }
            break;

        case UTC:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_CHAR4) > 0) {
                info.date_time_utc.hour = canasMessage.data.container.CHAR4[0];
                info.date_time_utc.minute = canasMessage.data.container.CHAR4[1];
                info.date_time_utc.second = canasMessage.data.container.CHAR4[2];
                info.time = TimeLocal(info.date_time_utc.GetSecondOfDay(), RoughTimeDelta()); // todo -- verify !!
                info.time_available.Update(info.clock);
                return true;
            }
            break;

        case HEADING_ANGLE:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                float value = canasMessage.data.container.FLOAT;
                if (value < 0.0) {
                    value += 360.0;
                }
                info.heading = Angle::Degrees(value);
                info.heading_available.Update(info.clock);
                return true;
            }
            break;

        case GPS_TRUE_TRACK:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.track = Angle::Degrees(canasMessage.data.container.FLOAT);
                info.track_available.Update(info.clock);
                return true;
            }
            break;

        case INDICATED_AIRSPEED:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.indicated_airspeed = canasMessage.data.container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case TRUE_AIRSPEED:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.true_airspeed = canasMessage.data.container.FLOAT;
                info.airspeed_available.Update(info.clock);
                info.airspeed_real = true;
                return true;
            }
            break;

        case GPS_GROUND_SPEED:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ground_speed = canasMessage.data.container.FLOAT;
                info.ground_speed_available.Update(info.clock);
                return true;
            }
            break;

        case AIRMASS_SPEED_VERTICAL:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideNettoVario(canasMessage.data.container.FLOAT);
                return true;
            }
            break;

        case STATIC_PRESSURE:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideStaticPressure(AtmosphericPressure::HectoPascal(canasMessage.data.container.FLOAT));
                return true;
            }
            break;

        case STANDARD_ALTITUDE:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideBaroAltitudeTrue(canasMessage.data.container.FLOAT);
                return true;
            }
            break;

        case FLARM_STATE_ID:  // Flarm messages: PFLAU
            // PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>, <RelativeVertical>,<RelativeDistance>
            switch (canasMessage.service_code) {
                case 2:
                    if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_USHORT2) == 0) return false;
                    break;
                case 3:
                    if (canasNetworkToHost(&canasMessage.data, canData, 2, CANAS_DATATYPE_SHORT) == 0) return false;
                    break;
                default:
                    if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_UCHAR4) == 0) return false;
            }
            if (canasFlarmStatePropagated(&canasMessage, info.gps_altitude, &objectData, &flarmState)) {
                info.flarm.status.available.Update(info.clock);
                info.flarm.status.rx = flarmState.RxDevicesCount;
                info.flarm.status.tx = flarmState.TxState;
                info.flarm.status.gps = (FlarmStatus::GPSStatus) flarmState.GpsState;
                info.flarm.status.alarm_level = (FlarmTraffic::AlarmType) objectData.AlarmLevel; // TODO -- correct ???
                return true;
            }
            break;

        case FLARM_OBJECT_AL3_ID: // Flarm messages: PFLAA
        case FLARM_OBJECT_AL2_ID:
        case FLARM_OBJECT_AL1_ID:
        case FLARM_OBJECT_AL0_ID:
            // PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<ID-Type>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<Type>
            switch (canasMessage.service_code & 0x0f) {
                case 0:
                case 1:
                    if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_USHORT2) == 0) return false;
                    break;
                default:
                    if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_UCHAR4) == 0) return false;
            }
            if (canasFlarmObjectPropagated(&canasMessage, canFrame->can_id, &flarmObjectData)) {
                info.flarm.traffic.modified.Update(info.clock);
                FlarmTraffic traffic{};
                traffic.alarm_level = (FlarmTraffic::AlarmType) flarmObjectData.AlarmLevel; // TODO --verify
                traffic.relative_north = flarmObjectData.RelNorth;
                traffic.relative_east = flarmObjectData.RelEast;
                traffic.relative_altitude = RoughAltitude(flarmObjectData.RelVertical);
                traffic.distance = RoughDistance(flarmObjectData.RelHorizontal);
                traffic.id.Set(flarmObjectData.ID);
                //traffic.IdType = flarmObjectData.IdType; // todo -- does not exist yes !!
                traffic.track_received = flarmObjectData.valid.track;
                traffic.track = RoughAngle(Angle::Degrees(flarmObjectData.Track));
                traffic.turn_rate_received = flarmObjectData.valid.turnRate;
                traffic.turn_rate = flarmObjectData.TurnRate;
                traffic.speed_received = flarmObjectData.valid.groundSpeed;
                traffic.speed = RoughSpeed(flarmObjectData.GroundSpeed);
                traffic.climb_rate_received = flarmObjectData.valid.climbRate;
                if (!traffic.climb_rate_received) {
                    // Field is empty in stealth mode
                    //stealth = true;
                    traffic.climb_rate = 0.0;
                } else {
                    traffic.climb_rate = flarmObjectData.valid.climbRate / 100.0;
                }

                traffic.stealth = false;
                if (flarmObjectData.Type > 15 || flarmObjectData.Type == 14) {
                    traffic.type = FlarmTraffic::AircraftType::UNKNOWN;
                } else {
                    traffic.type = (FlarmTraffic::AircraftType) flarmObjectData.Type;
                }

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