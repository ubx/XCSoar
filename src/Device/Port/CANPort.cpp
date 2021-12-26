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

#include "CANPort.hpp"
#include "net/SocketError.hxx"
#include "net/if.h"
#include "event/Call.hxx"
#include "sys/socket.h"
#include <sys/ioctl.h>
#include <linux/can.h>

CANPort::CANPort(EventLoop &event_loop, const TCHAR *port_name,
                 PortListener *_listener, DataHandler &_handler)
                 : BufferedPort(_listener, _handler), socket(event_loop, BIND_THIS_METHOD(OnSocketReady))
{
  UniqueSocketDescriptor s;

  if (!s.Create(AF_CAN, SOCK_RAW, CAN_RAW))
    throw MakeSocketError("Failed to create socket");

  struct ifreq ifr;
  strcpy(ifr.ifr_name, port_name);
  int ret = ioctl(s.Get(), SIOCGIFINDEX, &ifr);

  if (ret != 0)
    throw FormatErrno("Can not connect to %s", ifr.ifr_name);

  struct sockaddr_can addr = {0};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  ret = bind(s.Get(), (struct sockaddr *) &addr, sizeof(addr));
  if (ret != 0)
    throw FormatErrno("Failed binding socket %s", ifr.ifr_name);

  socket.Open(s.Release());

  BlockingCall(event_loop, [this](){
    socket.ScheduleRead();
  });
}

CANPort::~CANPort() noexcept
{
  BlockingCall(GetEventLoop(), [this]()
  {
    socket.Close();
  });
}

PortState
CANPort::GetState() const noexcept
{
  if (socket.IsDefined())
    return PortState::READY;
  else
    return PortState::FAILED;
}

size_t
CANPort::Write(const void *data, size_t length)
{
  if (!socket.IsDefined())
    return 0;

  ssize_t nbytes = socket.GetSocket().Write(data, length);
  if (nbytes < 0)
    // TODO check EAGAIN?
    return 0;

  return nbytes;
}

void
CANPort::OnSocketReady(unsigned) noexcept
try {
  std::byte input[sizeof(struct can_frame)];
  ssize_t nbytes;
  nbytes = socket.GetSocket().Read(input, sizeof(input));
  if (nbytes < 0)
    throw MakeSocketError("Failed to receive");

  if (nbytes == 0) {
    socket.Close();
    StateChanged();
    return;
  }

  DataReceived({input, std::size_t(nbytes)});
} catch (...) {
  socket.Close();
  StateChanged();
  Error(std::current_exception());
}

//    // todo -- temporary, to be moved to CANaerospace.cpp!
//    const std::vector<uint32_t> can_ids{
//            INDICATED_AIRSPEED, TRUE_AIRSPEED,
//            HEADING_ANGLE,
//            STANDARD_ALTITUDE,
//            STATIC_PRESSURE,
//            AIRMASS_SPEED_VERTICAL,
//            GPS_AIRCRAFT_LATITUDE,
//            GPS_AIRCRAFT_LONGITUDE,
//            GPS_AIRCRAFT_HEIGHTABOVE_ELLIPSOID,
//            GPS_GROUND_SPEED,
//            GPS_TRUE_TRACK,
//            UTC,
//            FLARM_STATE_ID,
//            FLARM_OBJECT_AL3_ID,
//            FLARM_OBJECT_AL2_ID,
//            FLARM_OBJECT_AL1_ID,
//            FLARM_OBJECT_AL0_ID
//    };
//    ret = SetFilter(can_ids);

//int
//CANPort::SetFilter(const std::vector<uint32_t> &can_ids) {
//    can_filter rfilter[can_ids.size()];
//    for (size_t i = 0; i < can_ids.size(); i++) {
//        rfilter[i].can_id = can_ids[i];
//        rfilter[i].can_mask = CAN_SFF_MASK;
//    }
//    return setsockopt(uniqueSocketDescriptor.Get() , SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
//}