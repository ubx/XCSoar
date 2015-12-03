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

#include "../Request.hpp"
#include "Version.hpp"
#include "Java/Global.hxx"
#include "Java/String.hxx"
#include "Java/InputStream.hxx"
#include "Java/URL.hxx"
#include "Java/Exception.hxx"

#include <assert.h>

Net::Request::Request(Session &_session, const TCHAR *url,
                      unsigned timeout_ms)
  :env(Java::GetEnv())
{
  Java::String j_url(env, url);
  jobject url_object = Java::URL::Create(env, j_url);
  if (Java::DiscardException(env)) {
    connection = nullptr;
    input_stream = nullptr;
    return;
  }

  connection = Java::URL::openConnection(env, url_object);
  env->DeleteLocalRef(url_object);
  if (Java::DiscardException(env)) {
    connection = nullptr;
    input_stream = nullptr;
    return;
  }

  Java::URLConnection::setConnectTimeout(env, connection, (jint)timeout_ms);
  Java::URLConnection::setReadTimeout(env, connection, (jint)timeout_ms);

  input_stream = Java::URLConnection::getInputStream(env, connection);
  if (Java::DiscardException(env)) {
    env->DeleteLocalRef(connection);
    connection = nullptr;
    input_stream = nullptr;
    return;
  }
}

Net::Request::~Request()
{
  if (connection != nullptr)
    env->DeleteLocalRef(connection);

  if (input_stream != nullptr) {
    Java::InputStream::close(env, input_stream);
    env->ExceptionClear();

    env->DeleteLocalRef(input_stream);
  }
}

bool
Net::Request::Send(unsigned _timeout_ms)
{
  return input_stream != nullptr;
}

int64_t
Net::Request::GetLength() const
{
  assert(connection != nullptr);
  assert(input_stream != nullptr);

  return Java::URLConnection::getContentLength(env, connection);
}

ssize_t
Net::Request::Read(void *buffer, size_t buffer_size, unsigned timeout_ms)
{
  assert(connection != nullptr);
  assert(input_stream != nullptr);

  Java::URLConnection::setReadTimeout(env, connection, (jint)timeout_ms);

  Java::LocalRef<jbyteArray> array(env,
                                   (jbyteArray)env->NewByteArray(buffer_size));
  jint nbytes = Java::InputStream::read(env, input_stream, array.Get());
  if (Java::DiscardException(env))
    return -1;

  if (nbytes <= 0)
    return 0;

  env->GetByteArrayRegion(array.Get(), 0, nbytes, (jbyte *)buffer);
  return nbytes;
}
