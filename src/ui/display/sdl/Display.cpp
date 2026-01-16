// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Display.hpp"
#include "lib/fmt/RuntimeError.hxx"
#include "Asset.hpp"

#include <SDL.h>
#include <SDL_hints.h>
#include <cstdlib>

namespace SDL {
static void logAvailableVideoDrivers(const char* phase)
{
  SDL_Log("=== SDL video drivers (%s) ===", phase);

  int count = SDL_GetNumVideoDrivers();
  if (count <= 0) {
    SDL_Log("No SDL video drivers available");
    return;
  }

  for (int i = 0; i < count; ++i) {
    SDL_Log("  %d: %s", i, SDL_GetVideoDriver(i));
  }
}

Display::Display()
{
  // SDL logging works even before SDL_Init()
  SDL_LogSetAllPriority(SDL_LOG_PRIORITY_VERBOSE);

  logAvailableVideoDrivers("before SDL_Init");

  // First try: prefer X11
  setenv("SDL_VIDEODRIVER", "x11", 0);

  if (SDL_Init(SDL_INIT_VIDEO) == 0) {
    SDL_Log("SDL video initialized using driver: %s",
            SDL_GetCurrentVideoDriver());
    return;
  }

  SDL_LogError(SDL_LOG_CATEGORY_VIDEO,
               "SDL_Init(x11) failed: %s", SDL_GetError());

  // Clean up any partial state
  SDL_QuitSubSystem(SDL_INIT_VIDEO);

  // Second try: fallback to dummy
  setenv("SDL_VIDEODRIVER", "dummy", 1);

  if (SDL_Init(SDL_INIT_VIDEO) == 0) {
    SDL_Log("SDL video initialized using driver: %s",
            SDL_GetCurrentVideoDriver());
    return;
  }

  SDL_LogError(SDL_LOG_CATEGORY_VIDEO,
               "SDL_Init(dummy) failed: %s", SDL_GetError());

  throw FmtRuntimeError("SDL_Init() failed: {}", SDL_GetError());
}


Display::~Display() noexcept
{
  ::SDL_Quit();
}

} // namespace SDL
