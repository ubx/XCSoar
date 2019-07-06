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

#include <boost/asio/ip/icmp.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <boost/asio/detail/push_options.hpp>

namespace boost {
namespace asio {

class Can
{
public:
    /// The type of a CAN endpoint.
    typedef boost::asio::ip::basic_endpoint<Can> endpoint;

    /// The type of a resolver query.
    typedef boost::asio::ip::basic_resolver_query<Can> resolver_query;

    /// The type of a resolver iterator.
    typedef boost::asio::ip::basic_resolver_iterator<Can>
        resolver_iterator;

    /// Construct to represent the socket CAN protocol.
    Can()
        : protocol_(CAN_RAW), family_(PF_CAN)
    {
    }

    static Can v4()
    {
        return Can();
    }

    static Can v6()
    {
        return Can();
    }

    /// Obtain an identifier for the type of the protocol.
    int type() const
    {
        return SOCK_RAW;
    }

    /// Obtain an identifier for the protocol.
    int protocol() const
    {
        return protocol_;
    }

    /// Obtain an identifier for the protocol family.
    int family() const
    {
        return family_;
    }

    /// The CAN socket type.
    typedef boost::asio::basic_raw_socket<Can> socket;

    /// The CAN resolver type.
    typedef boost::asio::ip::basic_resolver<Can> resolver;

    /// Compare two protocols for equality.
    friend bool operator==(const Can &p1, const Can &p2)
    {
        return p1.protocol_ == p2.protocol_ && p1.family_ ==
                                                   p2.family_;
    }

    /// Compare two protocols for inequality.
    friend bool operator!=(const Can &p1, const Can &p2)
    {
        return p1.protocol_ != p2.protocol_ || p1.family_ !=
                                                   p2.family_;
    }

private:
  // Construct with a specific family.
  explicit Can(int protocol_id, int protocol_family)
    : protocol_(protocol_id),
      family_(protocol_family)
  {
  }

  int protocol_;
  int family_;
};

} // namespace asio
} // namespace boost



/**
 * A CAN listener port class.
 */
class CANPort final : public BufferedPort
{
  boost::asio::Can::socket socket;


  char input[4096];

public:
  /**
   * Creates a new CANPort object, but does not open it yet.
   *
   * @param handler the callback object for input received on the
   * port
   */
  CANPort(boost::asio::io_context &io_context,
          unsigned port,
          PortListener *_listener, DataHandler &_handler);

  /**
   * Closes the serial port (Destructor)
   */
  virtual ~CANPort();

  /* virtual methods from class Port */
  PortState GetState() const override;

  bool Drain() override {
    /* writes are synchronous */
    return true;
  }

  bool SetBaudrate(unsigned baud_rate) override {
    return true;
  }

  unsigned GetBaudrate() const override {
    return 0;
  }

  size_t Write(const void *data, size_t length) override;

protected:
  void AsyncRead() {
    socket.async_receive(boost::asio::buffer(input, sizeof(input)),
                         std::bind(&CANPort::OnRead, this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));
  }

  void OnRead(const boost::system::error_code &ec, size_t nbytes);
};

#endif
