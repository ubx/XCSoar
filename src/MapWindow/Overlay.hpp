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

#ifndef XCSOAR_MAP_OVERLAY_HPP
#define XCSOAR_MAP_OVERLAY_HPP

#include "Compiler.h"

#include <tchar.h>

class Canvas;
class WindowProjection;
struct GeoPoint;

/**
 * An overlay that can be rendered in the #MapWindow.
 *
 * @see MapWindow:SetOverlay()
 */
class MapOverlay {
public:
  /**
   * Returns a human-readable name for this overlay.
   */
  gcc_pure
  virtual const TCHAR *GetLabel() const = 0;

  /**
   * Check whether the given location is inside the overlay.
   */
  gcc_pure
  virtual bool IsInside(GeoPoint p) const = 0;

  /**
   * Draw the overlay to the given #Canvas.
   */
  virtual void Draw(Canvas &canvas,
                    const WindowProjection &projection) noexcept = 0;
};

#endif
