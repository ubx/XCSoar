// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "CANPort.hpp"
#include "net/SocketError.hxx"
#include "net/if.h"
#include "event/Call.hxx"
#include "sys/socket.h"
#include "system/Error.hxx"
#include <sys/ioctl.h>
#include <linux/can.h>
#include <cstring>
#include <cstdio>

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

  if (ret != 0) {
    char msg[128];
    std::snprintf(msg, sizeof(msg), "Can not connect to %s", ifr.ifr_name);
    throw MakeErrno(msg);
  }

  sockaddr_can addr = {0};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  ret = bind(s.Get(), (struct sockaddr *) &addr, sizeof(addr));
  if (ret != 0) {
    char msg[128];
    std::snprintf(msg, sizeof(msg), "Failed binding socket %s", ifr.ifr_name);
    throw MakeErrno(msg);
  }

  socket.Open(s.Release());

  BlockingCall(event_loop, [this](){
    socket.ScheduleRead();
  });
}

CANPort::~CANPort()
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

std::size_t
CANPort::Write(std::span<const std::byte> src)
{
  if (!socket.IsDefined())
    return 0;

  // Expect full CAN frame writes; reject unexpected sizes to avoid partial/invalid frames
  if (src.size() != sizeof(struct can_frame))
    return 0;

  ssize_t nbytes = socket.GetSocket().Write(src);
  if (nbytes < 0)
    return 0;

  return static_cast<std::size_t>(nbytes);
}

void
CANPort::OnSocketReady(unsigned) noexcept
try {
  can_frame input;
  auto dest = std::as_writable_bytes(std::span{&input, std::size_t(1)});
  ssize_t nbytes = socket.GetSocket().Read(dest);
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return; // spurious readiness, try again later
    throw MakeSocketError("Failed to receive");
  }

  if (nbytes == 0) {
    socket.Close();
    StateChanged();
    return;
  }

  auto bytes = std::as_bytes(std::span{&input, std::size_t(1)});
  bytes = bytes.first(std::size_t(nbytes));
  DataReceived(bytes);
} catch (...) {
  socket.Close();
  StateChanged();
  Error(std::current_exception());
}