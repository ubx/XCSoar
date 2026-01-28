// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InputQueue.hpp"
#include "../shared/Event.hpp"
#include "DisplayOrientation.hpp"

namespace UI {

InputEventQueue::InputEventQueue(EventQueue &queue) noexcept
  :
#if defined(KOBO) || defined(COLIBRI)
   keyboard(queue, merge_mouse),
   mouse(queue, merge_mouse),
   mouse2(queue, merge_mouse)
#else
   libinput_handler(queue)
#endif
{
#ifdef KOBO
  /* power button */
  keyboard.Open("/dev/input/event0");

  /* Kobo touch screen */
  mouse.Open("/dev/input/event1");
#elif  defined(COLIBRI)
  /* power button */
  keyboard.Open("/dev/input/event5");
  /* Colibri touch screen */
  mouse.Open("/dev/input/event6");
  /* external mouse */
  mouse2.Open("/dev/input/event8");
#else
  libinput_handler.Open();
#endif
}

InputEventQueue::~InputEventQueue() noexcept = default;

bool
InputEventQueue::Generate([[maybe_unused]] Event &event) noexcept
{
#if defined(KOBO) || defined(COLIBRI)
  event = merge_mouse.Generate();
  if (event.type != Event::Type::NOP)
    return true;
#endif

  return false;
}

} // namespace UI
