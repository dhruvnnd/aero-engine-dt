#include "platform/sdl/sdl_input.h"

/* Seconds of held key to travel the full closed-to-open range. */
#define SDL_INPUT_THROTTLE_RATE_PER_S 0.5

/* Stick deflection (0..1) inside which the axis reads as centred. */
#define SDL_INPUT_STICK_DEADZONE 0.25

/* Right-trigger pull (0..1) past which it takes absolute command of the
 * lever. Below this the rate controls stay live. */
#define SDL_INPUT_TRIGGER_THRESHOLD 0.05

/* Signed axis sample -> -1.0 .. 1.0. */
static double axis_frac(Sint16 value) {
  return (double)value / (double)SDL_JOYSTICK_AXIS_MAX;
}

/* Binds `id` unless a gamepad is already held. */
static void try_bind(SdlInputState *input, SDL_JoystickID id) {
  if (input->pad) {
    return;
  }
  input->pad = SDL_OpenGamepad(id);
  input->pad_id = input->pad ? id : 0;
}

/* Binds the first gamepad SDL currently lists, if any. */
static void bind_first_available(SdlInputState *input) {
  int count = 0;
  SDL_JoystickID *ids = SDL_GetGamepads(&count);
  if (!ids) {
    return;
  }
  if (count > 0) {
    try_bind(input, ids[0]);
  }
  SDL_free(ids);
}

void sdl_input_init(SdlInputState *input) {
  input->throttle = 0.0;
  input->pad = NULL;
  input->pad_id = 0;

  if (!SDL_InitSubSystem(SDL_INIT_GAMEPAD)) {
    SDL_Log("Couldn't initialize SDL gamepad subsystem: %s", SDL_GetError());
    return;
  }

  bind_first_available(input);
}

void sdl_input_shutdown(SdlInputState *input) {
  if (input->pad) {
    SDL_CloseGamepad(input->pad);
    input->pad = NULL;
    input->pad_id = 0;
  }
}

void sdl_input_handle_event(SdlInputState *input, const SDL_Event *event) {
  switch (event->type) {
    case SDL_EVENT_GAMEPAD_ADDED:
      try_bind(input, event->gdevice.which);
      break;
    case SDL_EVENT_GAMEPAD_REMOVED:
      if (input->pad && event->gdevice.which == input->pad_id) {
        sdl_input_shutdown(input);
        bind_first_available(input);
      }
      break;
    default:
      break;
  }
}

void sdl_input_update(SdlInputState *input, double dt) {
  const bool *keys = SDL_GetKeyboardState(NULL);
  SDL_Gamepad *pad = input->pad;

  bool snap_open = keys[SDL_SCANCODE_HOME];
  bool snap_closed = keys[SDL_SCANCODE_END];

  double direction = 0.0;
  if (keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_W]) {
    direction += 1.0;
  }
  if (keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_S]) {
    direction -= 1.0;
  }

  if (pad) {
    if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER)) {
      snap_open = true;
    }
    if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER)) {
      snap_closed = true;
    }
    if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_DPAD_UP)) {
      direction += 1.0;
    }
    if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_DPAD_DOWN)) {
      direction -= 1.0;
    }

    /* Left stick: up (negative axis) opens, down closes. */
    double stick = -axis_frac(SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_LEFTY));
    if (stick > SDL_INPUT_STICK_DEADZONE) {
      direction += 1.0;
    } else if (stick < -SDL_INPUT_STICK_DEADZONE) {
      direction -= 1.0;
    }
  }

  if (snap_open) {
    input->throttle = 1.0;
    return;
  }
  if (snap_closed) {
    input->throttle = 0.0;
    return;
  }

  /* A pulled right trigger commands the lever position outright; releasing it
   * hands control back to the rate inputs from wherever the lever now sits. */
  if (pad) {
    double trigger =
        axis_frac(SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER));
    if (trigger > SDL_INPUT_TRIGGER_THRESHOLD) {
      input->throttle = (trigger > 1.0) ? 1.0 : trigger;
      return;
    }
  }

  /* Several sources can stack; hold the travel rate to a single lever's worth
   * so the feel matches the keyboard-only path. */
  if (direction > 1.0) {
    direction = 1.0;
  } else if (direction < -1.0) {
    direction = -1.0;
  }

  input->throttle += direction * SDL_INPUT_THROTTLE_RATE_PER_S * dt;

  if (input->throttle < 0.0) {
    input->throttle = 0.0;
  }
  if (input->throttle > 1.0) {
    input->throttle = 1.0;
  }
}
