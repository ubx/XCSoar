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

#include <Device/Driver/CANaerospace/canaerospace/message.h> // todo -- temporary, to be removed !
#include "CANPort.hpp"
#include "OS/Error.hxx"

CANPort::CANPort(boost::asio::io_context &io_context,
                 PortListener *_listener, DataHandler &_handler)
  :BufferedPort(_listener, _handler),
    socket_(io_context) {}

CANPort::~CANPort()
{
  BufferedPort::BeginClose();

  if (socket_.is_open())
    CancelWait(socket_);

  BufferedPort::EndClose();
}

bool
CANPort::Open(const char *port_name, unsigned baud_rate) {

  sc = socket( PF_CAN, SOCK_RAW, CAN_RAW );

  struct ifreq ifr;
  strcpy(ifr.ifr_name, port_name);
  int ret = ioctl(sc, SIOCGIFINDEX, &ifr);

  if(ret != 0){
    throw FormatErrno("Can not connect to %s", ifr.ifr_name);
  }

   // todo -- temporary, to be moved to CANaerospace.cpp!
  const std::vector<uint32_t> can_ids{
            INDICATED_AIRSPEED, TRUE_AIRSPEED,
            HEADING_ANGLE,
            STANDARD_ALTITUDE,
            STATIC_PRESSURE,
            AIRMASS_SPEED_VERTICAL,
            GPS_AIRCRAFT_LATITUDE,
            GPS_AIRCRAFT_LONGITUDE,
            GPS_AIRCRAFT_HEIGHTABOVE_ELLIPSOID,
            GPS_GROUND_SPEED,
            GPS_TRUE_TRACK,
            UTC};
  ret = SetFilter(can_ids);
  if(ret != 0){
      boost::system::error_code(ret,boost::system::system_category());
      close(sc);
      return false;
  }

  struct sockaddr_can addr = {0};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind( sc, (struct sockaddr*)&addr, sizeof(addr) );

  if(ret != 0){
    throw FormatErrno("Failed binding socket %s", ifr.ifr_name );
  }

  socket_.assign(sc);
  AsyncRead();
  StateChanged();
  return true;
}

PortState
CANPort::GetState() const
{
  if (socket_.is_open())
    return PortState::READY;
  else
    return PortState::FAILED;
}

size_t
CANPort::Write(const void *data, size_t length)
{
  if (!socket_.is_open())
    return 0;

  boost::system::error_code ec;
  size_t nbytes = socket_.write_some(boost::asio::buffer(data, length));
  if (ec)
    nbytes = 0;

  return nbytes;
}

void
CANPort::OnRead(const boost::system::error_code &ec, size_t nbytes)
{
  if (ec == boost::asio::error::operation_aborted)
    /* this object has already been deleted; bail out quickly without
       touching anything */
    return;

  if (ec) {
    socket_.close();
    StateChanged();
    Error(ec.message().c_str());
    return;
  }
  DataReceived(&input, nbytes);

  AsyncRead();
}

void
CANPort::AsyncRead() 
  {
    socket_.async_read_some(boost::asio::buffer(&input, sizeof(input)),
                         std::bind(&CANPort::OnRead, this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));
  }

int
CANPort::SetFilter(const std::vector<uint32_t>& can_ids)
  {
    can_filter rfilter[can_ids.size()];
    for (size_t i = 0; i < can_ids.size(); i++) {
        rfilter[i].can_id   = can_ids[i];
        rfilter[i].can_mask = CAN_SFF_MASK;
    }
    return setsockopt(sc, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
  }