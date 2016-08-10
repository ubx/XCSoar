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

#include "Audio/PCMPlayer.hpp"
#include "Audio/PCMPlayerFactory.hpp"
#include "Audio/VarioSynthesiser.hpp"
#include "Screen/Init.hpp"
#include "IO/Async/GlobalAsioThread.hpp"
#include "IO/Async/AsioThread.hpp"
#include "OS/Sleep.h"
#include "OS/Args.hpp"
#include "DebugReplay.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <memory>

int
main(int argc, char **argv)
{
  Args args(argc, argv, "DRIVER FILE");
  DebugReplay *replay = CreateDebugReplay(args);
  if (replay == NULL)
    return EXIT_FAILURE;

  args.ExpectEnd();

  ScreenGlobalInit screen;

  ScopeGlobalAsioThread global_asio_thread;
  std::unique_ptr<PCMPlayer> player(
      PCMPlayerFactory::CreateInstanceForDirectAccess(asio_thread->Get()));

  const unsigned sample_rate = 44100;

  VarioSynthesiser synthesiser(sample_rate);

  if (!player->Start(synthesiser)) {
    fprintf(stderr, "Failed to start PCMPlayer\n");
    return EXIT_FAILURE;
  }

  while (replay->Next()) {
    auto vario = replay->Basic().brutto_vario;
    printf("%2.1f\n", (double)vario);
    synthesiser.SetVario(vario);
    Sleep(1000);
  }

  return EXIT_SUCCESS;
}
