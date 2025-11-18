// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Device/Driver/CANaerospace.hpp"
#include "Device/Driver.hpp"
#include "Device/Config.hpp"
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
#include <Geo/Gravity.hpp>
#include <cstdio>

class CANaerospaceDevice final : public AbstractDevice {
//  Port &port;

public:
    CANaerospaceDevice([[maybe_unused]] Port &_port) {}

    bool DataReceived(std::span<const std::byte> s, NMEAInfo &info) noexcept override;
};

std::map<int, double> canId2clock;
auto last_fix = GeoPoint::Invalid();
SpeedVector last_wind = SpeedVector::Zero();
static FlarmState flarmState;
static FlarmMostImportantObjectData objectData;
static FlarmObjectData flarmObjectData;
static bool slcan_port = false;

static Device *
CANaerospaceCreateOnPort([[maybe_unused]] const DeviceConfig &config, Port &com_port)
{
    slcan_port = config.port_type == DeviceConfig::PortType::ANDROID_USB_SERIAL;
    if (slcan_port) {
        com_port.Write("C\n");
        com_port.Write("S6\n");
        com_port.Write("O\n");
        com_port.Drain();
        slcan_port = true;
    }
    return new CANaerospaceDevice(com_port);
}

bool
CANaerospaceDevice::DataReceived(std::span<const std::byte> s,
                                 NMEAInfo &info) noexcept {

    if (s.size() < sizeof(can_frame))
        return false;
    /*
     * todo:
     *  1. check if we receive a SLCAN message
     *  2. parse SLCAN message and create a canFrame.
     */
    if (slcan_port) {
      // todo -- implement parse SLCAN message and create a canFrame.
    }

    const auto *canFrame = reinterpret_cast<const can_frame *>(s.data());   // Cast the address to a can frame
    const auto *canData = canFrame->data + 4;
    const CanasMessage *cm = reinterpret_cast<const CanasMessage *>(canFrame->data);

    static CanasMessage canasMessage;
    canasMessage.message_code = cm->message_code;
    canasMessage.service_code = cm->service_code;

    static double qnh_corr = 0.0;
    static double last_body_long_acc = 0.0;
    static double last_body_lat_acc = 0.0;
    static double last_body_norm_acc = 0.0;

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
                info.time = TimeStamp{info.date_time_utc.DurationSinceMidnight()};
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
                info.attitude.heading = Angle::Degrees(value);
                info.attitude.heading_available.Update(info.clock);
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
                info.ProvideIndicatedAirspeed(canasMessage.data.container.FLOAT);
                return true;
            }
            break;

        case TRUE_AIRSPEED:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideTrueAirspeed(canasMessage.data.container.FLOAT);
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
                info.ProvideTotalEnergyVario(-canasMessage.data.container.FLOAT);
                return true;
            }
            break;

        case STATIC_PRESSURE: // todo -- verify!
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideStaticPressure(AtmosphericPressure::Pascal(canasMessage.data.container.FLOAT));
                //std::cout << "STATIC_PRESSURE [Pascal]=" << info.static_pressure.GetPascal() << std::endl;
                return true;
            }
            break;

        case STANDARD_ALTITUDE: // todo -- verify!
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.ProvideBaroAltitudeTrue(canasMessage.data.container.FLOAT + qnh_corr);
                //std::cout << "QNH_CORR [m]=" << qnh_corr << std::endl;
                // std::cout << "STANDARD_ALTITUDE [m]=" << info.baro_altitude << std::endl;
                return true;
            }
            break;

        case BARO_CORRECTION_ID: /* QNH */
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.settings.ProvideQNH(AtmosphericPressure::Pascal(canasMessage.data.container.FLOAT), info.clock);
                //std::cout << "BARO_CORRECTION_ID QNH-1 [Hp]=" << info.settings.qnh.GetHectoPascal() << std::endl;
                info.settings.qnh_available.Update(info.clock);
                return true;
            }
            break;

        case BARO_ALT_CORR_ID:  // todo -- verify
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                switch (canasMessage.service_code & 0x0f) {
                case 0: /* QNH */
                   qnh_corr = canasMessage.data.container.FLOAT;
                   //std::cout << "BARO_ALT_CORR_ID QNH [m]=" << qnh_corr << std::endl;
                   return true;

                case 1: /* QFE */
                   //std::cout << "BARO_ALT_CORR_ID QFE [m]=" << canasMessage.data.container.FLOAT << std::endl;
                  return true;

                default:
                   break;
                }
            }
        break;

        case BODY_LAT_ACC_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                last_body_lat_acc = canasMessage.data.container.FLOAT;
                info.acceleration.ProvideGLoad(SpaceDiagonal(last_body_lat_acc, last_body_long_acc, last_body_norm_acc) / GRAVITY);
                return true;
            }
            break;

        case BODY_LONG_ACC_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                last_body_long_acc = canasMessage.data.container.FLOAT;
                info.acceleration.ProvideGLoad(SpaceDiagonal(last_body_lat_acc, last_body_long_acc, last_body_norm_acc) / GRAVITY);
                return true;
            }
            break;

        case BODY_NORM_ACC_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                last_body_norm_acc = canasMessage.data.container.FLOAT;
                info.acceleration.ProvideGLoad(SpaceDiagonal(last_body_lat_acc, last_body_long_acc, last_body_norm_acc) / GRAVITY);
                return true;
            }
            break;

        case WIND_SPEED_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                last_wind.norm = canasMessage.data.container.FLOAT;
                info.ProvideExternalWind(last_wind.Reciprocal());
                return true;
            }
            break;

        case WIND_DIRECTION_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                last_wind.bearing = Angle::Degrees(canasMessage.data.container.FLOAT);
                info.ProvideExternalWind(last_wind.Reciprocal());
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
                {
                    char id_hex[16];
                    std::snprintf(id_hex, sizeof(id_hex), "%X", flarmObjectData.ID);
                    traffic.id = FlarmId::Parse(id_hex, nullptr);
                }
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

        case MCCRADY_VALUE_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.settings.ProvideMacCready(canasMessage.data.container.FLOAT, info.clock);
                return true;
            }
        break;

        case VARIO_MODE_ID: // todo -- check if datatype is correct
            if (canasNetworkToHost(&canasMessage.data, canData, 2, CANAS_DATATYPE_SHORT) > 0) {
                /* NOTE: works only if the Glide Computer has Flap force cruise ON  */
                info.switch_state.flight_mode = (canasMessage.data.container.SHORT == 0)
                                              ? SwitchState::FlightMode::CIRCLING : SwitchState::FlightMode::CRUISE;
                return true;
            }
        break;

        case OUTSIDE_AIR_TEMP_ID:
            if (canasNetworkToHost(&canasMessage.data, canData, 4, CANAS_DATATYPE_FLOAT) > 0) {
                info.temperature_available = true;
                info.temperature = Temperature::FromKelvin(canasMessage.data.container.FLOAT);
                return true;
            }
        break;

        default:
            // std::cout << "not implemented can_id: " << canFrame->can_id << std::endl;
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
