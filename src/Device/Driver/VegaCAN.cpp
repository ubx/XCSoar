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
#include <Device/Driver/VegaCAN/marshal.h>

class VegaCANDevice : public AbstractDevice {
  Port &port;

  public:
    VegaCANDevice(Port &_port):port(_port) {}

    bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;

    GeoPoint last_fix = GeoPoint::Invalid();

};


static void 
PrintData(const can_frame* frame) {
  for (int i=0;i<frame->can_dlc && i < 8; ++i){
    std::cout << "data[" << i << "]: "  << static_cast<unsigned int>(frame->data[i]) << std::endl;
  }
}

static Device *
VegaCANCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new VegaCANDevice(com_port);
}

bool
VegaCANDevice::DataReceived(const void *data, size_t length,
                           NMEAInfo &info) {
    assert(data != nullptr);
    assert(length > 0);


    const can_frame *canFrame = (const can_frame *) data;   // Cast the adress to a can frame
    std::cout << "CAN ID: " << canFrame->can_id << std::endl;

    PrintData(canFrame);
    CanasMessageData *phost = new(CanasMessageData);

    switch (canFrame->can_id) {
        case 1036: // Latitude
            if (canasNetworkToHost(phost, canFrame->data + 4, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.latitude = Angle::Native((double) phost->container.LONG / 1E7);
            }
            return false;

        case 1037:  // Longitude
            if (canasNetworkToHost(phost, canFrame->data + 4, 4, CANAS_DATATYPE_LONG) > 0) {
                last_fix.longitude = Angle::Native((double) phost->container.LONG / 1E7);
                if (last_fix.IsValid()) {
                    info.location = last_fix;
                    info.location_available.Update(info.clock); //todo -- investigate "src/Engine/Contest/Solvers/Retrospective.cpp:78: bool Retrospective::UpdateSample(const GeoPoint&): Assertion `aircraft_location.IsValid()' failed"
                    info.alive.Update(info.clock);
                    return true;
                }
            }
            return false;

        case 1038: // Height
            if (canasNetworkToHost(phost, canFrame->data + 4, 4, CANAS_DATATYPE_FLOAT) > 0) { // todo -- explain the "+4" !!
                info.gps_altitude = phost->container.FLOAT;
                info.gps_altitude_available.Update(info.clock);
                info.alive.Update(info.clock);
                return true;
            }
    }
    return false;
}

const struct DeviceRegister vega_can_driver = {
  _T("VegaCAN"),
  _T("VegaCAN"),
  DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA, // TODO: Put the right flags
  VegaCANCreateOnPort,
};
