#ifndef UI_GAMEPAD_PANEL_H
#define UI_GAMEPAD_PANEL_H

#include <SDL3/SDL.h>

void gamepad_panel_draw(SDL_Renderer *r, float w, float h, SDL_Gamepad *pad,
                        double throttle);

#endif /* UI_GAMEPAD_PANEL_H */
