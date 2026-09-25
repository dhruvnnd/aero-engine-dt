#ifndef UI_CONTROLS_PANEL_H
#define UI_CONTROLS_PANEL_H

#include "model/state.h"
#include "platform/sdl/sdl_input.h"

/* What the user asked for this frame; the caller performs it. */
struct ControlActions {
  bool start_engine;
  bool stop_engine;
};

/* Dockable mouse controls for everything the keyboard and gamepad drive:
 * engine start / stop, throttle, altitude, airspeed and OAT offset sliders,
 * a flight-condition reset, and the display-feed switch. Sliders write
 * straight into `input`; `*sensor_mode` is 1 for the noisy sensor feed, 0 for
 * the exact model. `open` is cleared when the user closes the window. */
ControlActions controls_panel_draw(bool *open, SdlInputState *input,
                                   const ModelState *s, int *sensor_mode);

#endif /* UI_CONTROLS_PANEL_H */
