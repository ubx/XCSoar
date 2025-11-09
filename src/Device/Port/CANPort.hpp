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

#ifndef XCSOAR_DEVICE_CAN_PORT_HPP
#define XCSOAR_DEVICE_CAN_PORT_HPP

#include "BufferedPort.hpp"
#include "event/SocketEvent.hxx"
#include "net/UniqueSocketDescriptor.hxx"
#include "unix/tchar.h"

/**
 * A CAN listener port class.
 */
class CANPort final : public BufferedPort
{
  SocketEvent socket;

public:
  /**
   * Creates a new CANPort object, but does not open it yet.
   *
   * @param handler the callback object for input received on the
   * port
   */
  CANPort(EventLoop &event_loop, const TCHAR *port_name,
          PortListener *_listener, DataHandler &_handler);
  /**
   * Closes the serial port (Destructor)
   */
  virtual ~CANPort();

  auto &GetEventLoop() const noexcept {
    return socket.GetEventLoop();
  }

  /* virtual methods from class Port */
  PortState GetState() const noexcept override;

  bool Drain() override {
    /* writes are synchronous */
    return true;
  }

  void SetBaudrate(unsigned /*baud_rate*/) override {}

  unsigned GetBaudrate() const noexcept override {
    return 0;
  }

  std::size_t Write(std::span<const std::byte> src) override;

protected:
  void OnSocketReady(unsigned events) noexcept;
};

#endif
