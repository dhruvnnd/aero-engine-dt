#ifndef UI_GAMEPAD_PANEL_H
#define UI_GAMEPAD_PANEL_H

#include <SDL3/SDL.h>

#include "platform/sdl/sdl_input.h"

void gamepad_panel_draw(SDL_Renderer *r, float w, float h,
                        const SdlInputState *input);

#endif /* UI_GAMEPAD_PANEL_H */
