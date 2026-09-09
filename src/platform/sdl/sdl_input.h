#ifndef PLATFORM_SDL_SDL_INPUT_H
#define PLATFORM_SDL_SDL_INPUT_H

#include <SDL3/SDL.h>

typedef struct {
  double throttle; /* current lever position, 0.0 (closed) .. 1.0 (WOT) */
} SdlInputState;

/* Sets the lever to fully closed. */
void sdl_input_init(SdlInputState *input);

/* Reads the current keyboard state and moves `throttle` toward its held
 * target by `dt` seconds' worth of lever travel, then clamps to [0, 1].
 *
 *   Up   / W  -- open throttle
 *   Down / S  -- close throttle
 *   Home      -- snap fully open
 *   End       -- snap fully closed
 *
 * Call once per frame. SDL_AppIterate runs after event delivery, so the
 * keyboard snapshot is already current there. */
void sdl_input_update(SdlInputState *input, double dt);

#endif /* PLATFORM_SDL_SDL_INPUT_H */
