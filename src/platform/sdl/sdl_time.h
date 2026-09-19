#ifndef PLATFORM_SDL_SDL_TIME_H
#define PLATFORM_SDL_SDL_TIME_H

#ifdef __cplusplus
extern "C" {
#endif
#include <SDL3/SDL.h>

typedef struct {
  Uint64 last_ticks;
  float fps;
} SdlFrameTimer;

float sdl_time_tick(SdlFrameTimer *timer);

#ifdef __cplusplus
}
#endif

#endif // !PLATFORM_SDL_SDL_TIME_H
