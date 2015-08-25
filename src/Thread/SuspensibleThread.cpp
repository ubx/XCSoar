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

#include "Thread/SuspensibleThread.hpp"

#include <assert.h>

bool
SuspensibleThread::Start(bool _suspended)
{
  stop_received = false;
  suspend_received = _suspended;
  suspended = false;

  return Thread::Start();
}

void
SuspensibleThread::BeginStop()
{
  mutex.lock();
  stop_received = true;
  command_trigger.signal();
  mutex.unlock();
}

void
SuspensibleThread::BeginSuspend()
{
  assert(!Thread::IsInside());
  assert(Thread::IsDefined());

  mutex.lock();
  suspend_received = true;
  command_trigger.signal();
  mutex.unlock();
}

void
SuspensibleThread::WaitUntilSuspended()
{
  assert(!Thread::IsInside());
  assert(Thread::IsDefined());

  mutex.lock();
  assert(suspend_received);

  while (!suspended)
    client_trigger.wait(mutex);
  mutex.unlock();
}

void
SuspensibleThread::Suspend()
{
  BeginSuspend();
  WaitUntilSuspended();
}

void
SuspensibleThread::Resume()
{
  mutex.lock();
  suspend_received = false;
  command_trigger.signal();
  mutex.unlock();
}

bool
SuspensibleThread::IsCommandPending()
{
  assert(Thread::IsInside());

  mutex.lock();
  bool result = stop_received || suspend_received;
  mutex.unlock();
  return result;
}

bool
SuspensibleThread::CheckStoppedOrSuspended()
{
  assert(Thread::IsInside());

  mutex.lock();

  assert(!suspended);

  if (!stop_received && suspend_received) {
    suspended = true;
    client_trigger.signal();
    while (!stop_received && suspend_received)
      command_trigger.wait(mutex);
    suspended = false;
  }

  bool stop = stop_received;
  mutex.unlock();
  return stop;
}

bool
SuspensibleThread::WaitForStopped(unsigned timeout_ms)
{
  assert(Thread::IsInside());

  mutex.lock();

  assert(!suspended);
  suspended = true;

  if (!stop_received)
    command_trigger.timed_wait(mutex, timeout_ms);

  if (!stop_received && suspend_received) {
    client_trigger.signal();
    while (!stop_received && suspend_received)
      command_trigger.wait(mutex);
  }

  suspended = false;
  bool stop = stop_received;
  mutex.unlock();
  return stop;
}
