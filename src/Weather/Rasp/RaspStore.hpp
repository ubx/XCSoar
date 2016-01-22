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

#ifndef XCSOAR_WEATHER_RASP_STORE_HPP
#define XCSOAR_WEATHER_RASP_STORE_HPP

#include "Util/StaticArray.hxx"
#include "Util/StaticString.hxx"
#include "Time/BrokenTime.hpp"
#include "Compiler.h"

#include <assert.h>
#include <tchar.h>

class Path;
class RasterMap;
struct GeoPoint;
struct zzip_dir;

/**
 * Class to manage raster weather data.  Usually, these raster maps
 * are generated by RASP.
 */
class RaspStore {
public:
  static constexpr unsigned MAX_WEATHER_MAP = 16; /**< Max number of items stored */
  static constexpr unsigned MAX_WEATHER_TIMES = 48; /**< Max time segments of each item */

  struct MapInfo {
    const TCHAR *name;

    /**
     * Human-readable label.  Call gettext() for internationalization.
     */
    const TCHAR *label;

    /**
     * Human-readable help text.  Call gettext() for
     * internationalization.
     */
    const TCHAR *help;
  };

  struct MapItem {
    StaticString<32> name;

    /**
     * Human-readable label.  Call gettext() for internationalization.
     */
    const TCHAR *label;

    /**
     * Human-readable help text.  Call gettext() for
     * internationalization.
     */
    const TCHAR *help;

    bool times[MAX_WEATHER_TIMES];

    MapItem() = default;
    explicit MapItem(const TCHAR *_name);
  };

  typedef StaticArray<MapItem, MAX_WEATHER_MAP> MapList;

private:
  /**
   * Not protected by #lock because it's written only by ScanAll()
   * during startup.
   */
  MapList maps;

public:
  gcc_const
  unsigned GetItemCount() const {
    return maps.size();
  }

  gcc_const
  const MapItem &GetItemInfo(unsigned i) const {
    return maps[i];
  }

  /**
   * Load a list of RASP maps from the file "xcsoar-rasp.dat".
   */
  void ScanAll();

  bool IsTimeAvailable(unsigned item_index, unsigned time_index) const {
    assert(item_index < maps.size());
    assert(time_index < MAX_WEATHER_TIMES);

    return maps[item_index].times[time_index];
  }

  template<typename C>
  void ForEachTime(unsigned item_index, C &&c) {
    assert(item_index < maps.size());

    const auto &mi = maps[item_index];

    for (unsigned i = 0; i < MAX_WEATHER_TIMES; ++i)
      if (mi.times[i])
        c(IndexToTime(i));
  }

  /**
   * Find the nearest time index which is available.  If no time index
   * is available, this method returns #MAX_WEATHER_TIMES.
   */
  gcc_pure
  unsigned GetNearestTime(unsigned item_index, unsigned time_index) const;

  /**
   * Converts a time index to a #BrokenTime.
   */
  gcc_const
  static BrokenTime IndexToTime(unsigned index);

  static struct zzip_dir *OpenArchive();

  static bool NarrowWeatherFilename(char *filename, Path name,
                                    unsigned time_index);

private:
  gcc_pure
  static bool ExistsItem(struct zzip_dir *dir, Path name,
                         unsigned time_index);

  static bool ScanMapItem(struct zzip_dir *dir, MapItem &item);
};

#endif
