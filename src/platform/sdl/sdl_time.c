#include "platform/sdl/sdl_time.h"

float sdl_time_tick(SdlFrameTimer *timer) {
  const Uint64 curr_ticks = SDL_GetTicks();
  const Uint64 frame_time = curr_ticks - timer->last_ticks;

  if (frame_time > 0) {
    timer->fps = 1000.0f / (float)frame_time;
  }

  timer->last_ticks = curr_ticks;
  return timer->fps;
}
