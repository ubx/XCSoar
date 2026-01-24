// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#ifdef SOFTWARE_ROTATE_DISPLAY
#include "ui/canvas/opengl/Globals.hpp"
#endif

#include "ui/canvas/custom/TopCanvas.hpp"
#include "ui/canvas/Features.hpp"
#include "ui/dim/Size.hpp"
#include "ui/canvas/memory/Export.hpp"
#include "ui/canvas/memory/Buffer.hpp"
#include "lib/fmt/RuntimeError.hxx"
#include "Asset.hpp"

#ifdef ENABLE_OPENGL
#include "ui/dim/Rect.hpp"
#include "ui/canvas/opengl/Init.hpp"
#include "Math/Point2D.hpp"
#include "LogFile.hpp"
#else
#include "ui/canvas/Canvas.hpp"
#endif

#ifdef DITHER
#include "ui/canvas/memory/Dither.hpp"
#endif

#include <SDL_platform.h>
#include <SDL_video.h>
#include <SDL_hints.h>
#ifdef USE_MEMORY_CANVAS
#include <SDL_render.h>
#endif
#if defined(__MACOSX__) && __MACOSX__
#include <SDL_syswm.h>
#import <AppKit/AppKit.h>
#include <alloca.h>
#endif

#if defined(__APPLE__) && TARGET_OS_IPHONE
#import <UIKit/UIKit.h>
#endif

#include <cassert>

#ifdef ENABLE_OPENGL

[[gnu::pure]]
static int
GetConfigAttrib(SDL_GLattr attribute, int default_value) noexcept
{
  int value;
  return SDL_GL_GetAttribute(attribute, &value) == 0
    ? value
    : default_value;
}

#endif

TopCanvas::TopCanvas(UI::Display &_display, SDL_Window *_window)
  :display(_display), window(_window)
{
#ifdef USE_MEMORY_CANVAS
  renderer = SDL_CreateRenderer(window, -1, 0);
  if (renderer == nullptr)
    throw FmtRuntimeError("SDL_CreateRenderer({}, {}, {}) has failed: {}",
                          (const void *)window, -1, 0, ::SDL_GetError());

  int width, height;
  SDL_GetRendererOutputSize(renderer, &width, &height);
  texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB888,
                              SDL_TEXTUREACCESS_STREAMING,
                              width, height);
  if (texture == nullptr)
    throw FmtRuntimeError("SDL_CreateTexture({}, {}, {}, {}, {}) has failed: {}",
                          (const void *)renderer,
                          (unsigned)SDL_PIXELFORMAT_UNKNOWN,
                          (unsigned)SDL_TEXTUREACCESS_STREAMING,
                          width, height,
                          ::SDL_GetError());
#endif

#ifdef ENABLE_OPENGL
  if (::SDL_GL_CreateContext(window) == nullptr)
    throw FmtRuntimeError("SDL_GL_CreateContext({}) has failed: {}",
                          (const void *)window, ::SDL_GetError());

  LogFormat("SDL_GL config: RGB=%d/%d/%d alpha=%d depth=%d stencil=%d",
            GetConfigAttrib(SDL_GL_RED_SIZE, 0),
            GetConfigAttrib(SDL_GL_GREEN_SIZE, 0),
            GetConfigAttrib(SDL_GL_BLUE_SIZE, 0),
            GetConfigAttrib(SDL_GL_ALPHA_SIZE, 0),
            GetConfigAttrib(SDL_GL_DEPTH_SIZE, 0),
            GetConfigAttrib(SDL_GL_STENCIL_SIZE, 0));

  /* this is usually done by OpenGL::Display, but libSDL doesn't allow
     that */
  OpenGL::SetupContext();

  SetupViewport(GetNativeSize());
#endif

#ifdef USE_MEMORY_CANVAS
  buffer.Allocate(PixelSize(width, height));
#endif
}

TopCanvas::~TopCanvas() noexcept
{
#ifdef USE_MEMORY_CANVAS
  buffer.Free();
#endif

#ifdef USE_MEMORY_CANVAS
  SDL_DestroyTexture(texture);
#endif
}

#ifdef ENABLE_OPENGL

PixelSize
TopCanvas::GetNativeSize() const noexcept
{
  int w, h;
  SDL_GL_GetDrawableSize(window, &w, &h);
  return PixelSize(w, h);
}

#else

PixelSize
TopCanvas::GetNativeSize() const noexcept
{
  int w, h;
  SDL_GetWindowSize(window, &w, &h);
  return PixelSize(w, h);
}

#endif

#ifdef USE_MEMORY_CANVAS

void
TopCanvas::OnResize(PixelSize new_size) noexcept
{
  int texture_width, texture_height;
  Uint32 texture_format;
  if (SDL_QueryTexture(texture, &texture_format, NULL, &texture_width, &texture_height) != 0)
    return;
  if ((int)new_size.width == texture_width && (int)new_size.height == texture_height)
    return;

  SDL_Texture *t = SDL_CreateTexture(renderer, texture_format,
                                     SDL_TEXTUREACCESS_STREAMING,
                                     new_size.width, new_size.height);
  if (t == nullptr)
    return;

  if (texture != nullptr)
      SDL_DestroyTexture(texture);
  texture = t;

  buffer.Free();
  buffer.Allocate(new_size);
}

#endif // USE_MEMORY_CANVAS

#ifndef ENABLE_OPENGL

Canvas
TopCanvas::Lock()
{
#ifdef USE_MEMORY_CANVAS
  return Canvas(buffer);
#else
  return Canvas();
#endif
}

void
TopCanvas::Unlock() noexcept
{
}

#endif

void
TopCanvas::Flip()
{
#ifdef ENABLE_OPENGL
  ::SDL_GL_SwapWindow(window);
#else
  void* pixels;
  int pitch;
  if (SDL_LockTexture(texture, nullptr, &pixels, &pitch) == 0) {
    Uint32 format;
    int w, h;
    SDL_QueryTexture(texture, &format, nullptr, &w, &h);
    unsigned bpp = SDL_BYTESPERPIXEL(format);

#ifdef GREYSCALE
    CopyFromGreyscale(
#ifdef DITHER
                      dither,
#endif
                      pixels, pitch, bpp, buffer, orientation);
#else
    CopyFromBGRA(pixels, pitch, bpp, buffer, orientation);
#endif

    SDL_UnlockTexture(texture);
  }

  ::SDL_RenderCopy(renderer, texture, nullptr, nullptr);
  ::SDL_RenderPresent(renderer);
#endif
}

#ifdef SOFTWARE_ROTATE_DISPLAY
PixelSize
TopCanvas::SetDisplayOrientation(DisplayOrientation _orientation) noexcept
{
  if (orientation == _orientation)
    return GetSize();

  const bool was_swapped = AreAxesSwapped(orientation);
  const bool now_swapped = AreAxesSwapped(_orientation);

  orientation = _orientation;
  OpenGL::display_orientation = orientation;

  if (was_swapped != now_swapped) {
    PixelSize size = buffer.size;
    buffer.Free();
    buffer.Allocate(size.Swapped());
  }

  return GetSize();
}
#endif

#if defined(__APPLE__) && TARGET_OS_IPHONE

bool
TopCanvas::IsIOSAppActive() const noexcept
{
  // Check if the iOS app is in an active state where rendering is appropriate
  // Attempting to render while the app is in the background will crash the app
  UIApplicationState appState = [[UIApplication sharedApplication] applicationState];
  
  switch (appState) {
    case UIApplicationStateActive:
      // App is active and in foreground - safe to render
      return true;
      
    case UIApplicationStateInactive:
      // App is transitioning between states - avoid rendering
      return false;
      
    case UIApplicationStateBackground:
      // App is in background - definitely don't render
      return false;
      
    default:
      // Unknown state - we are conservative and don't render
      return false;
  }
}

#endif
