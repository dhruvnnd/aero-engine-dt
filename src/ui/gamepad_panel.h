#ifndef UI_GAMEPAD_PANEL_H
#define UI_GAMEPAD_PANEL_H

#include "platform/sdl/sdl_input.h"

/* Draws the gamepad / input-map panel as a dockable ImGui window. `open` is
 * cleared when the user closes the window. Call between NewFrame/Render. */
void gamepad_panel_draw(bool *open, const SdlInputState *input);

#endif /* UI_GAMEPAD_PANEL_H */
