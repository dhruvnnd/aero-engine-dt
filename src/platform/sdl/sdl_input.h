#ifndef PLATFORM_SDL_SDL_INPUT_H
#define PLATFORM_SDL_SDL_INPUT_H

#include <SDL3/SDL.h>

typedef struct {
  double throttle;     /* current lever position, 0.0 (closed) .. 1.0 (WOT) */
  double altitude_m;   /* pressure altitude, m; 0 .. 12000 */
  double airspeed_ms;  /* true airspeed, m/s; 0 .. 120 */
  double oat_offset_c; /* OAT above ISA, degC; -40 .. 50 */

  SDL_Gamepad *pad;      /* first bound gamepad, or NULL if none attached */
  SDL_JoystickID pad_id; /* instance id backing `pad`, for hotplug matching */
} SdlInputState;

/* Sets the lever to fully closed, the flight condition to sea-level still
 * air, brings up SDL's gamepad subsystem, and binds the first gamepad
 * already attached (if any). */
void sdl_input_init(SdlInputState *input);

/* Releases any bound gamepad. Safe to call when none is held. */
void sdl_input_shutdown(SdlInputState *input);

/* Feeds SDL gamepad connect/disconnect events so a controller plugged in
 * after startup gets picked up, and a yanked one dropped (falling back to
 * any other still-attached pad). Ignores every other event type. Call from
 * SDL_AppEvent. */
void sdl_input_handle_event(SdlInputState *input, const SDL_Event *event);

/* Reads the current keyboard + gamepad state and moves `throttle` toward its
 * held target by `dt` seconds' worth of lever travel, then clamps to [0, 1].
 *
 *   Up / W ...... D-pad Up,   left stick up ..... open throttle
 *   Down / S .... D-pad Down, left stick down ... close throttle
 *   Home ........ right shoulder ................ snap fully open
 *   End ......... left shoulder ................. snap fully closed
 *               ( right trigger ) .............. hold the lever at the
 *                                                trigger's own position
 *   Page Up/Down ... right stick Y (up +) ........ climb / descend
 *   [ / ] .......... right stick X (right +) ..... slow down / speed up
 *   - / = .......... D-pad Left / Right ........... OAT offset down / up
 *   R .............. Back ......................... reset flight condition
 *
 * Call once per frame. SDL_AppIterate runs after event delivery, so both
 * input snapshots are already current there. */
void sdl_input_update(SdlInputState *input, double dt);

#endif /* PLATFORM_SDL_SDL_INPUT_H */
