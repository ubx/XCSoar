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
#include "OS/Error.hxx"

CANPort::CANPort(boost::asio::io_context &io_context,
                 PortListener *_listener, DataHandler &_handler)
  :BufferedPort(_listener, _handler),
    socket_(io_context) // toodo -- ???
{
}

CANPort::~CANPort()
{
  BufferedPort::BeginClose();

  if (socket_.is_open())
    CancelWait(socket_);

  BufferedPort::EndClose();
}

bool
CANPort::Open(const char *port_name, unsigned baud_rate) {


  int sc = socket( PF_CAN, SOCK_RAW, CAN_RAW );

  struct ifreq ifr;
  strcpy(ifr.ifr_name, port_name);
  int ret = ioctl(sc, SIOCGIFINDEX, &ifr);

  if(ret != 0){
    throw FormatErrno("Can not connect to %s", ifr.ifr_name);
    return false;
  }

  /* CAN Fram filter options  */
//  struct can_filter rfilter[6];
//    rfilter[0].can_id   = 0x40C;
//    rfilter[0].can_mask = CAN_SFF_MASK;
//    rfilter[1].can_id   = 0x40D;
//    rfilter[1].can_mask = CAN_SFF_MASK;
//    rfilter[2].can_id   = 0x40E ;
//    rfilter[2].can_mask = CAN_SFF_MASK;
//    rfilter[3].can_id   = 0x4B0 ;
//    rfilter[3].can_mask = CAN_SFF_MASK;
//    rfilter[4].can_id   = 0x141 ;
//    rfilter[4].can_mask = CAN_SFF_MASK;
//    rfilter[5].can_id   = 0x13B ;
//    rfilter[5].can_mask = CAN_SFF_MASK;
//
//  ret = setsockopt(sc, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  //if(ret != 0){
  //    boost::system::error_code(ret,boost::system::system_category());
  //    close(sc);
  //    return false;
  //}

  struct sockaddr_can addr = {0};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind( sc, (struct sockaddr*)&addr, sizeof(addr) );

  if(ret != 0){
    throw FormatErrno("Failed binding socket %s", ifr.ifr_name );
    return false;
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
