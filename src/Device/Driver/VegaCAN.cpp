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
#include "Device/Util/NMEAWriter.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Checksum.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream> // TODO: Remove this, its just for debugging


class VegaCANDevice : public AbstractDevice {
  Port &port;

  public:
    VegaCANDevice(Port &_port):port(_port) {}

    bool DataReceived(const void *data, size_t length, NMEAInfo &info) override;
};

static signed int 
ByteToInt(const can_frame* frame)
{
     signed int i = frame->data[3] | (frame->data[2] << 8) | (frame->data[1] << 16) | (frame->data[0] << 24);
     return i;
}

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
                           NMEAInfo &info)
{
  assert(data != nullptr);
  assert(length > 0);
  

  const can_frame* data_ = (const can_frame*) data;   // Cast the adress to a can frame
  std::cout << "CAN ID: " <<  data_->can_id << std::endl;

  // I guess the parsing part should go here:
  GeoPoint location;

  switch (data_->can_id) {
    case 1036: 
      PrintData(data_);
      ByteToInt(data_);
      info.location = location;
      info.location_available.Update(info.clock);
      return true;

    case 1037: PrintData(data_); return true; // Lon
    case 1038: 
      PrintData(data_);
      info.gps_altitude = ByteToInt(data_);
      info.gps_altitude_available.Update(info.clock);
      info.alive.Update(info.clock);
      return true; // Height
  }
  
  return true;
}

const struct DeviceRegister vega_can_driver = {
  _T("VegaCAN"),
  _T("VegaCAN"),
  DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA, // TODO: Put the right flags
  VegaCANCreateOnPort,
};
