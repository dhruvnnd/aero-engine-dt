#include "ui/gamepad_panel.h"

#include "ui/ui_draw.h"
#include "ui/ui_layout.h"
#include "ui/ui_theme.h"
#include "ui/ui_widgets.h"

static const UiRange RANGE_STICK = {.lo = -1.0, .hi = 1.0};
static const UiRange RANGE_UNIT = {.lo = 0.0, .hi = 1.0};

static const char *const BIND_ROWS[][3] = {
    {"R-TRIGGER", "hold throttle at trigger pos", ""},
    {"L-STICK UP", "open throttle", "Up / W"},
    {"D-PAD UP", "open throttle", "Up / W"},
    {"L-STICK DN", "close throttle", "Dn / S"},
    {"D-PAD DN", "close throttle", "Dn / S"},
    {"R-SHOULDER", "snap fully open", "Home"},
    {"L-SHOULDER", "snap fully closed", "End"},
    {"Y / NORTH", "toggle sensor / model feed", "M"},
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

void gamepad_panel_draw(SDL_Renderer *r, float w, float h, SDL_Gamepad *pad,
                        double throttle) {
  const UiTheme th = ui_theme_default();

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
            "keyboard throttle control is active");
  }

  /* live input */
  UiRect livebox = ui_split_top(rest, 220.0f, 8.0f, &rest);
  UiRect live = ui_panel(r, livebox, &th, "live input");
  UiStack ls = ui_stack(live, 4.0f);
  ui_bar_gauge(r, ui_stack_row(&ls, 30.0f), &th, "L-STICK Y (up +)",
               -axis_unit(pad, SDL_GAMEPAD_AXIS_LEFTY), "", 2, RANGE_STICK);
  ui_bar_gauge(r, ui_stack_row(&ls, 30.0f), &th, "R-TRIGGER",
               axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), "", 2,
               RANGE_UNIT);
  ui_bar_gauge(r, ui_stack_row(&ls, 30.0f), &th, "-> THROTTLE OUT", throttle,
               "", 2, RANGE_UNIT);
  flag_row(r, ui_stack_row(&ls, th.row_h), &th, "left shoulder",
           held(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER));
  flag_row(r, ui_stack_row(&ls, th.row_h), &th, "right shoulder",
           held(pad, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER));
  flag_row(r, ui_stack_row(&ls, th.row_h), &th, "d-pad up",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_UP));
  flag_row(r, ui_stack_row(&ls, th.row_h), &th, "d-pad down",
           held(pad, SDL_GAMEPAD_BUTTON_DPAD_DOWN));
  flag_row(r, ui_stack_row(&ls, th.row_h), &th, "Y (north)",
           held(pad, SDL_GAMEPAD_BUTTON_NORTH));

  /* bindings */
  UiRect bind = ui_panel(r, rest, &th, "bindings");
  const float c1 = bind.x;
  const float c2 = bind.x + 12.0f * UI_GLYPH_W;
  const float c3 = bind.x + 42.0f * UI_GLYPH_W;
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
