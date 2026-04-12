// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InputQueue.hpp"
#include "../shared/Event.hpp"
#include "DisplayOrientation.hpp"

#ifndef USE_LIBINPUT
#include <cstdio>
#endif

namespace UI {

InputEventQueue::InputEventQueue(EventQueue &queue) noexcept
  :
#ifndef USE_LIBINPUT
   merge_mouse()
#else
   libinput_handler(queue)
#endif
{
#ifndef USE_LIBINPUT
#ifdef KOBO
  devices.emplace_front(queue, merge_mouse);
  /* power button */
  devices.front().Open("/dev/input/event0");

  devices.emplace_front(queue, merge_mouse);
  /* Kobo touch screen */
  devices.front().Open("/dev/input/event1");
#elif defined(COLIBRI)
  for (unsigned i = 0; i < 32; ++i) {
    char path[64];
    std::sprintf(path, "/dev/input/event%u", i);

    devices.emplace_front(queue, merge_mouse);
    if (!devices.front().Open(path))
      devices.pop_front();
  }

  /* open keyboard devices from /dev/input/by-id/ */
  static const char *const keyboard_devices[] = {
    "/dev/input/by-id/usb-Logitech_USB_Keyboard-event-kbd",
    "/dev/input/by-id/usb-Logitech_USB_Receiver-if01-event-kbd",
    "/dev/input/by-id/usb-04d9_USB_Keyboard-event-kbd",
  };

  for (const char *path : keyboard_devices) {
    devices.emplace_front(queue, merge_mouse);
    if (!devices.front().Open(path))
      devices.pop_front();
  }
#else
  for (unsigned i = 0; i < 32; ++i) {
    char path[64];
    std::sprintf(path, "/dev/input/event%u", i);

    devices.emplace_front(queue, merge_mouse);
    if (!devices.front().Open(path))
      devices.pop_front();
  }
#endif
#else
  libinput_handler.Open();
#endif
}

InputEventQueue::~InputEventQueue() noexcept = default;

bool
InputEventQueue::Generate([[maybe_unused]] Event &event) noexcept
{
#ifndef USE_LIBINPUT
  event = merge_mouse.Generate();
  if (event.type != Event::Type::NOP)
    return true;
#endif

  return false;
}

} // namespace UI
