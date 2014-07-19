/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2014 The XCSoar Project
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

#ifndef XCSOAR_DATA_FIELD_COMBO_LIST_HPP
#define XCSOAR_DATA_FIELD_COMBO_LIST_HPP

#include "Util/StaticArray.hpp"

#include <tchar.h>

class ComboList {
public:
  struct Item {
    static constexpr int NEXT_PAGE = -800001;
    static constexpr int PREVIOUS_PAGE = -800002;

    int int_value;
    TCHAR *string_value;
    TCHAR *display_string;
    TCHAR *help_text;

    Item(int _int_value, const TCHAR *_string_value,
         const TCHAR *_display_string, const TCHAR *_help_text = NULL);
    ~Item();

    Item(const Item &other) = delete;
    Item &operator=(const Item &other) = delete;
  };

#ifdef _WIN32_WCE
  static constexpr unsigned MAX_SIZE = 300;
#else
  static constexpr unsigned MAX_SIZE = 512;
#endif

  int current_index;

private:
  StaticArray<Item*, MAX_SIZE> items;

public:
  ComboList()
    :current_index(0) {}

  ComboList(ComboList &&other);

  ~ComboList() {
    Clear();
  }

  ComboList(const ComboList &other) = delete;
  ComboList &operator=(const ComboList &other) = delete;

  unsigned size() const {
    return items.size();
  }

  const Item& operator[](unsigned i) const {
    return *items[i];
  }

  void Clear();

  unsigned Append(Item *item);

  unsigned Append(int int_value,
                  const TCHAR *string_value,
                  const TCHAR *display_string,
                  const TCHAR *help_text = NULL) {
    return Append(new Item(int_value,
                           string_value, display_string, help_text));
  }

  unsigned Append(int int_value, const TCHAR *string_value) {
    return Append(int_value, string_value, string_value);
  }

  void Sort();
  unsigned LookUp(int int_value);
};

#endif
