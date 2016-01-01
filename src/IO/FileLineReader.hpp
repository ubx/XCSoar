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

#ifndef XCSOAR_IO_FILE_LINE_READER_HPP
#define XCSOAR_IO_FILE_LINE_READER_HPP

#include "FileSource.hpp"
#include "LineSplitter.hpp"
#include "ConvertLineReader.hpp"

/**
 * Glue class which combines FileSource and LineSplitter, and provides
 * a public NLineReader interface.
 */
class FileLineReaderA : public NLineReader {
protected:
  FileSource file;
  LineSplitter splitter;

public:
  FileLineReaderA(Path path, Error &error)
    :file(path, error), splitter(file) {}

  bool error() const {
    return file.error();
  }

  /**
   * Rewind the file to the beginning.
   */
  bool Rewind() {
    if (!file.Rewind())
      return false;

    splitter.ResetBuffer();
    return true;
  }

public:
  /* virtual methods from class NLineReader */
  char *ReadLine() override;
  long GetSize() const override;
  long Tell() const override;
};

/**
 * Glue class which combines FileSource, LineSplitter and
 * ConvertLineReader, and provides a public TLineReader interface.
 */
class FileLineReader : public TLineReader {
protected:
  FileSource file;
  LineSplitter splitter;
  ConvertLineReader convert;

public:
  FileLineReader(Path path, Error &error,
                 Charset cs=Charset::UTF8)
    :file(path, error), splitter(file), convert(splitter, cs) {}

  bool error() const {
    return file.error();
  }

  /**
   * Rewind the file to the beginning.
   */
  bool Rewind() {
    if (!file.Rewind())
      return false;

    splitter.ResetBuffer();
    return true;
  }

public:
  /* virtual methods from class TLineReader */
  TCHAR *ReadLine() override;
  long GetSize() const override;
  long Tell() const override;
};

#endif
