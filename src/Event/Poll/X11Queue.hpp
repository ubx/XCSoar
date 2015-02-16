/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
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

#ifndef XCSOAR_EVENT_X11_EVENT_QUEUE_HPP
#define XCSOAR_EVENT_X11_EVENT_QUEUE_HPP

#include "IO/Async/FileEventHandler.hpp"

class IOLoop;
class EventQueue;
struct Event;
struct _XDisplay;
union _XEvent;

/**
 * This class opens a connection to the X11 server using Xlib and
 * listens for events.
 */
class X11EventQueue final : FileEventHandler {
  IOLoop &io_loop;
  EventQueue &queue;

  _XDisplay *const display;

public:
  /**
   * @param io_loop the #IOLoop that shall be used to register the
   * Xlib socket
   * @param queue the #EventQueue that shall receive X11 events
   */
  X11EventQueue(IOLoop &io_loop, EventQueue &queue);

  ~X11EventQueue();

  _XDisplay *GetDisplay() const {
    return display;
  }

  bool Generate(Event &event) {
    return false;
  }

private:
  void HandleEvent(_XEvent &event);

  /* virtual methods from FileEventHandler */
  bool OnFileEvent(FileDescriptor fd, unsigned mask) override;
};

#endif
