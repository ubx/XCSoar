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

#include "Terrain/RasterTerrain.hpp"
#include "Profile/Profile.hpp"
#include "Compatibility/path.h"

#include <windef.h> /* for MAX_PATH */

#include <string.h>

RasterTerrain *
RasterTerrain::OpenTerrain(FileCache *cache, OperationEnvironment &operation)
{
  TCHAR path[MAX_PATH], world_file_buffer[MAX_PATH];
  const TCHAR *world_file;

  if (Profile::GetPath(ProfileKeys::MapFile, path)) {
    _tcscpy(world_file_buffer, path);
    _tcscat(world_file_buffer, _T(DIR_SEPARATOR_S "terrain.j2w"));
    world_file = world_file_buffer;

    _tcscat(path, _T("/terrain.jp2"));
  } else
    return nullptr;

  RasterTerrain *rt = new RasterTerrain(path, world_file, cache, operation);
  if (!rt->map.IsDefined()) {
    delete rt;
    return nullptr;
  }

  return rt;
}
