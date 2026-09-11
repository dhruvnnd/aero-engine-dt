#include "ui/gamepad_panel.h"

#include "ui/ui_draw.h"
#include "ui/ui_layout.h"
#include "ui/ui_theme.h"
#include "ui/ui_widgets.h"

/* Full-scale spans for the little bar read-outs (no caution/warning bands --
 * this window just mirrors the raw stick/trigger travel and the resolved
 * flight condition, not an operating limit). */
static const UiRange RANGE_STICK = {.lo = -1.0, .hi = 1.0};
static const UiRange RANGE_UNIT = {.lo = 0.0, .hi = 1.0};
static const UiRange RANGE_ALT = {.lo = 0.0, .hi = 12000.0};
static const UiRange RANGE_SPD = {.lo = 0.0, .hi = 120.0};
static const UiRange RANGE_OAT = {.lo = -40.0, .hi = 50.0};

/* control, action, keyboard equivalent. Mirrors sdl_input.c's read path plus
 * the sensor/model toggle handled in main.c's SDL_AppEvent. */
static const char *const BIND_ROWS[][3] = {
    {"R-TRIGGER", "hold throttle at trigger pos", ""},
    {"L-STICK UP", "open throttle", "Up / W"},
    {"D-PAD UP", "open throttle", "Up / W"},
    {"L-STICK DN", "close throttle", "Dn / S"},
    {"D-PAD DN", "close throttle", "Dn / S"},
    {"R-SHOULDER", "snap fully open", "Home"},
    {"L-SHOULDER", "snap fully closed", "End"},
    {"Y / NORTH", "toggle sensor / model feed", "M"},
    {"R-STICK UP", "climb", "Page Up"},
    {"R-STICK DN", "descend", "Page Dn"},
    {"R-STICK RIGHT", "speed up", "]"},
    {"R-STICK LEFT", "slow down", "["},
    {"D-PAD RIGHT", "OAT offset up", "="},
    {"D-PAD LEFT", "OAT offset down", "-"},
    {"BACK", "reset flight condition", "R"},
};

static double axis_unit(SDL_Gamepad *pad, SDL_GamepadAxis axis) {
  if (!pad) {
    return 0.0;
  }
  return (double)SDL_GetGamepadAxis(pad, axis) / (double)SDL_JOYSTICK_AXIS_MAX;
}

static bool held(SDL_Gamepad *pad, SDL_GamepadButton button) {
  return pad && SDL_GetGamepadButton(pad, button);
}

static void flag_row(SDL_Renderer *r, UiRect row, const UiTheme *th,
                     const char *label, bool on) {
  ui_text(r, row.x, row.y, th->text_dim, "%s", label);
  ui_text_right(r, row.x + row.w, row.y, on ? th->text_bright : th->text_dim,
                on ? "[ HELD ]" : "[  --  ]");
}

void gamepad_panel_draw(SDL_Renderer *r, float w, float h,
                        const SdlInputState *input) {
  const UiTheme th = ui_theme_default();
  SDL_Gamepad *pad = input->pad;

  ui_fill(r, (UiRect){0.0f, 0.0f, w, h}, th.bg);
  UiRect screen = ui_rect_inset((UiRect){0.0f, 0.0f, w, h}, 8.0f, 8.0f);

  UiRect body;
  UiRect header = ui_split_top(screen, 14.0f, 6.0f, &body);
  ui_text(r, header.x, header.y, th.text_bright, "gamepad  /  input map");
  ui_text_right(r, header.x + header.w, header.y, pad ? th.ok : th.warn,
                pad ? "CONNECTED" : "OFFLINE");
  ui_hline(r, screen.x, header.y + 12.0f, screen.w, th.frame);

  UiRect content;
  UiRect footer = ui_split_bottom(body, 10.0f, 6.0f, &content);
  ui_text(r, footer.x, footer.y, th.text_dim,
          "G  hide / show      close window to dismiss");

  /* device */
  UiRect rest;
  UiRect devbox = ui_split_top(content, 44.0f, 8.0f, &rest);
  UiRect dev = ui_panel(r, devbox, &th, "device");
  if (pad) {
    const char *name = SDL_GetGamepadName(pad);
    ui_text(r, dev.x, dev.y, th.text_bright, "%s",
            name ? name : "(unnamed controller)");
    ui_text(r, dev.x, dev.y + 10.0f, th.text_dim, "instance id %u",
            (unsigned)SDL_GetGamepadID(pad));
  } else {
    ui_text(r, dev.x, dev.y, th.warn, "no controller attached");
    ui_text(r, dev.x, dev.y + 10.0f, th.text_dim,
            "keyboard control is active");
  }

  /* throttle */
  UiRect throttlebox = ui_split_top(rest, 220.0f, 8.0f, &rest);
  UiRect thr = ui_panel(r, throttlebox, &th, "throttle input");
  UiStack ts = ui_stack(thr, 4.0f);
  ui_bar_gauge(r, ui_stack_row(&ts, 30.0f), &th, "L-STICK Y (up +)",
               -axis_unit(pad, SDL_GAMEPAD_AXIS_LEFTY), "", 2, RANGE_STICK);
  ui_bar_gauge(r, ui_stack_row(&ts, 30.0f), &th, "R-TRIGGER",
               axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), "", 2,
               RANGE_UNIT);
  ui_bar_gauge(r, ui_stack_row(&ts, 30.0f), &th, "-> THROTTLE OUT",
               input->throttle, "", 2, RANGE_UNIT);
  flag_row(r, ui_stack_row(&ts, th.row_h), &th, "left shoulder",
           held(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER));
  flag_row(r, ui_stack_row(&ts, th.row_h), &th, "right shoulder",
           held(pad, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER));
  flag_row(r, ui_stack_row(&ts, th.row_h), &th, "d-pad up",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_UP));
  flag_row(r, ui_stack_row(&ts, th.row_h), &th, "d-pad down",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_DOWN));
  flag_row(r, ui_stack_row(&ts, th.row_h), &th, "Y (north)",
           held(pad, SDL_GAMEPAD_BUTTON_NORTH));

  /* flight condition */
  UiRect envbox = ui_split_top(rest, 210.0f, 8.0f, &rest);
  UiRect env = ui_panel(r, envbox, &th, "flight condition input");
  UiStack es = ui_stack(env, 4.0f);
  ui_bar_gauge(r, ui_stack_row(&es, 30.0f), &th, "R-STICK Y (up +)",
               -axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHTY), "", 2, RANGE_STICK);
  ui_bar_gauge(r, ui_stack_row(&es, 30.0f), &th, "R-STICK X (right +)",
               axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHTX), "", 2, RANGE_STICK);
  flag_row(r, ui_stack_row(&es, th.row_h), &th, "d-pad right",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT));
  flag_row(r, ui_stack_row(&es, th.row_h), &th, "d-pad left",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_LEFT));
  flag_row(r, ui_stack_row(&es, th.row_h), &th, "back (reset)",
           held(pad, SDL_GAMEPAD_BUTTON_BACK));
  ui_reading_row(r, ui_stack_row(&es, th.row_h), &th, "-> altitude",
                input->altitude_m, "m", 0,
                ui_status_for(input->altitude_m, RANGE_ALT));
  ui_reading_row(r, ui_stack_row(&es, th.row_h), &th, "-> airspeed",
                input->airspeed_ms, "m/s", 1,
                ui_status_for(input->airspeed_ms, RANGE_SPD));
  ui_reading_row(r, ui_stack_row(&es, th.row_h), &th, "-> OAT offset",
                input->oat_offset_c, "degC", 1,
                ui_status_for(input->oat_offset_c, RANGE_OAT));

  /* bindings */
  UiRect bind = ui_panel(r, rest, &th, "bindings");
  const float c1 = bind.x;
  const float c2 = bind.x + 15.0f * UI_GLYPH_W;
  const float c3 = bind.x + 45.0f * UI_GLYPH_W;
  ui_text(r, c1, bind.y, th.text_dim, "CONTROL");
  ui_text(r, c2, bind.y, th.text_dim, "ACTION");
  ui_text(r, c3, bind.y, th.text_dim, "KEYS");
  ui_hline(r, bind.x, bind.y + 10.0f, bind.w, th.grid);
  float ry = bind.y + 14.0f;
  for (size_t i = 0; i < SDL_arraysize(BIND_ROWS); i++) {
    ui_text(r, c1, ry, th.text_bright, "%s", BIND_ROWS[i][0]);
    ui_text(r, c2, ry, th.text, "%s", BIND_ROWS[i][1]);
    ui_text(r, c3, ry, th.text_dim, "%s", BIND_ROWS[i][2]);
    ry += th.row_h;
  }
}
