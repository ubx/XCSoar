// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Init.hpp"
#include "Screen/Debug.hpp"
#include "ui/event/Globals.hpp"

#ifdef USE_FREETYPE
#include "ui/canvas/Font.hpp"
#endif

#if defined(KOBO) || defined(COLIBRI)
#include "Hardware/RotateDisplay.hpp"
#include "DisplayOrientation.hpp"
#endif

#ifdef USE_GDI
#include "ui/canvas/gdi/GdiPlusBitmap.hpp"
#endif

#ifdef USE_WINUSER
#include "PaintWindow.hpp"
#include "SingleWindow.hpp"

#include <libloaderapi.h>
#endif

ScreenGlobalInit::ScreenGlobalInit()
#ifdef ANDROID
  :display(EGL_DEFAULT_DISPLAY)
#endif
{
#ifdef USE_FREETYPE
  Font::Initialise();
#endif

#ifdef USE_GDI
  GdiStartup();
#endif

  UI::event_queue = &event_queue;

#if defined(KOBO) || defined(COLIBRI)
  Display::Rotate(DisplayOrientation::PORTRAIT); // todo
  UI::event_queue->SetDisplayOrientation(DisplayOrientation::PORTRAIT); // todo
#endif

#ifdef USE_WINUSER
  HINSTANCE hInstance = ::GetModuleHandle(nullptr);
  PaintWindow::register_class(hInstance);
  UI::SingleWindow::RegisterClass(hInstance);
#endif

  ScreenInitialized();
}

ScreenGlobalInit::~ScreenGlobalInit()
{
  UI::event_queue = nullptr;

#ifdef USE_GDI
  GdiShutdown();
#endif

#ifdef USE_FREETYPE
  Font::Deinitialise();
#endif

  ScreenDeinitialized();
}
