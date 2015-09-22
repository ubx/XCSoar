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

#include "IO/ZipLineReader.hpp"
#include "OS/Args.hpp"

#include <zzip/zzip.h>

#include <stdio.h>

int main(int argc, char **argv)
{
  Args args(argc, argv, "ZIPFILE FILENAME");
  const char *zip_path = args.ExpectNext();
  const char *filename = args.ExpectNext();
  args.ExpectEnd();

  auto dir = zzip_dir_open(zip_path, nullptr);
  if (dir == nullptr) {
    fprintf(stderr, "Failed to open %s\n", zip_path);
    return EXIT_FAILURE;
  }

  ZipLineReader reader(dir, filename);
  zzip_dir_close(dir);
  if (reader.error()) {
    fprintf(stderr, "Failed to open %s\n", filename);
    return EXIT_FAILURE;
  }

  TCHAR *line;
  while ((line = reader.ReadLine()) != NULL)
    _putts(line);

  return EXIT_SUCCESS;
}
