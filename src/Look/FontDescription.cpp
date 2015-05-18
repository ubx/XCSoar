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

#include "FontDescription.hpp"

#ifdef USE_GDI
#include "StandardFonts.hpp"
#include "Asset.hpp"

#include <string.h>

FontDescription::FontDescription(const TCHAR *face,
                                 unsigned height,
                                 bool bold, bool italic,
                                 bool monospace)
{
  Init(face, height, bold, italic, monospace);
}

FontDescription::FontDescription(unsigned height,
                                 bool bold, bool italic,
                                 bool monospace)
{
  Init(monospace ? GetStandardMonospaceFontFace() : GetStandardFontFace(),
       height, bold, italic, monospace);
}

void
FontDescription::Init(const TCHAR *face,
                      unsigned height,
                      bool bold, bool italic,
                      bool monospace)
{
  memset((char *)&logfont, 0, sizeof(logfont));

  _tcscpy(logfont.lfFaceName, face);

  logfont.lfPitchAndFamily = (monospace ? FIXED_PITCH : VARIABLE_PITCH)
    | FF_DONTCARE;

  logfont.lfHeight = (long)height;
  logfont.lfWeight = (long)(bold ? FW_BOLD : FW_MEDIUM);
  logfont.lfItalic = italic;
  logfont.lfCharSet = ANSI_CHARSET;

  if (IsAltair())
    logfont.lfQuality = NONANTIALIASED_QUALITY;
  else
    logfont.lfQuality = ANTIALIASED_QUALITY;
}

#endif
