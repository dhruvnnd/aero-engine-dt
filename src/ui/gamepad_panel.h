#ifndef UI_GAMEPAD_PANEL_H
#define UI_GAMEPAD_PANEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <SDL3/SDL.h>

#include "platform/sdl/sdl_input.h"

void gamepad_panel_draw(SDL_Renderer *r, float w, float h,
                        const SdlInputState *input);

#ifdef __cplusplus
}
#endif

#endif /* UI_GAMEPAD_PANEL_H */
