// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

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
