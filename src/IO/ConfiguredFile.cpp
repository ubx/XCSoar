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

#include "ConfiguredFile.hpp"
#include "MapFile.hpp"
#include "FileLineReader.hpp"
#include "ZipLineReader.hpp"
#include "Profile/Profile.hpp"
#include "LogFile.hpp"
#include "OS/Path.hpp"
#include "Util/Error.hxx"

#include <zzip/zzip.h>

#include <assert.h>
#include <string.h>

NLineReader *
OpenConfiguredTextFileA(const char *profile_key)
{
  assert(profile_key != nullptr);

  const auto path = Profile::GetPath(profile_key);
  if (path.IsNull())
    return nullptr;

  Error error;
  FileLineReaderA *reader = new FileLineReaderA(path, error);
  if (reader->error()) {
    delete reader;
    LogError(error);
    return nullptr;
  }

  return reader;
}

TLineReader *
OpenConfiguredTextFile(const char *profile_key, Charset cs)
{
  assert(profile_key != nullptr);

  const auto path = Profile::GetPath(profile_key);
  if (path.IsNull())
    return nullptr;

  Error error;
  FileLineReader *reader = new FileLineReader(path, error, cs);
  if (reader == nullptr) {
    LogError(error);
    return nullptr;
  }

  if (reader->error()) {
    delete reader;
    return nullptr;
  }

  return reader;
}

static TLineReader *
OpenMapTextFile(const char *in_map_file, Charset cs)
{
  assert(in_map_file != nullptr);

  auto dir = OpenMapFile();
  if (dir == nullptr)
    return nullptr;

  Error error;
  ZipLineReader *reader = new ZipLineReader(dir, in_map_file, error, cs);
  zzip_dir_close(dir);
  if (reader == nullptr) {
    LogError(error);
    return nullptr;
  }

  if (reader->error()) {
    delete reader;
    return nullptr;
  }

  return reader;
}

TLineReader *
OpenConfiguredTextFile(const char *profile_key, const char *in_map_file,
                       Charset cs)
{
  assert(profile_key != nullptr);
  assert(in_map_file != nullptr);

  TLineReader *reader = OpenConfiguredTextFile(profile_key, cs);
  if (reader == nullptr)
    reader = OpenMapTextFile(in_map_file, cs);

  return reader;
}
