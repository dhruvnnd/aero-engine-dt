#include "platform/sdl/sdl_input.h"

/* Seconds of held key to travel the full closed-to-open range. */
#define SDL_INPUT_THROTTLE_RATE_PER_S 0.5

void sdl_input_init(SdlInputState *input) { input->throttle = 0.0; }

void sdl_input_update(SdlInputState *input, double dt) {
  const bool *keys = SDL_GetKeyboardState(NULL);

  if (keys[SDL_SCANCODE_HOME]) {
    input->throttle = 1.0;
    return;
  }
  if (keys[SDL_SCANCODE_END]) {
    input->throttle = 0.0;
    return;
  }

  double direction = 0.0;
  if (keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_W]) {
    direction += 1.0;
  }
  if (keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_S]) {
    direction -= 1.0;
  }

  input->throttle += direction * SDL_INPUT_THROTTLE_RATE_PER_S * dt;

  if (input->throttle < 0.0) {
    input->throttle = 0.0;
  }
  if (input->throttle > 1.0) {
    input->throttle = 1.0;
  }
}
