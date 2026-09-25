#include "ui/gamepad_panel.h"

#include <float.h>
#include <stdio.h>

#include "imgui.h"

/* control, action, keyboard equivalent. */
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

static bool begin_rows(const char *id) {
  if (!ImGui::BeginTable(id, 2)) {
    return false;
  }
  ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 11.0f);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
  return true;
}

static void bar_row(const char *label, double v, double lo, double hi,
                    const char *fmt) {
  double frac = (v - lo) / (hi - lo);
  frac = frac < 0.0 ? 0.0 : (frac > 1.0 ? 1.0 : frac);
  char overlay[32];
  snprintf(overlay, sizeof overlay, fmt, v);

  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  ImGui::ProgressBar((float)frac, ImVec2(-FLT_MIN, 0.0f), overlay);
}

static void flag_row(const char *label, bool on) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  if (on) {
    ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.45f, 1.0f), "HELD");
  } else {
    ImGui::TextDisabled("--");
  }
}

static void reading_row(const char *label, double v, const char *unit,
                        const char *fmt) {
  char value[32];
  snprintf(value, sizeof value, fmt, v);

  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  ImGui::Text("%s %s", value, unit);
}

void gamepad_panel_draw(bool *open, const SdlInputState *input) {
  ImGui::SetNextWindowSize(ImVec2(460.0f, 640.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Gamepad / Input Map", open)) {
    ImGui::End();
    return;
  }

  SDL_Gamepad *pad = input->pad;

  if (pad) {
    ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.45f, 1.0f), "CONNECTED");
  } else {
    ImGui::TextColored(ImVec4(1.00f, 0.75f, 0.20f, 1.0f), "OFFLINE");
  }
  ImGui::SameLine();
  ImGui::TextDisabled("G  hide / show");

  if (ImGui::CollapsingHeader("Device", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (pad) {
      const char *name = SDL_GetGamepadName(pad);
      ImGui::TextUnformatted(name ? name : "(unnamed controller)");
      ImGui::TextDisabled("instance id %u", (unsigned)SDL_GetGamepadID(pad));
    } else {
      ImGui::TextColored(ImVec4(1.00f, 0.75f, 0.20f, 1.0f),
                         "no controller attached");
      ImGui::TextDisabled("keyboard control is active");
    }
  }

  if (ImGui::CollapsingHeader("Throttle input",
                              ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_rows("throttle")) {
    bar_row("L-STICK Y (up +)", -axis_unit(pad, SDL_GAMEPAD_AXIS_LEFTY), -1.0,
            1.0, "%+.2f");
    bar_row("R-TRIGGER", axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), 0.0,
            1.0, "%.2f");
    bar_row("-> THROTTLE OUT", input->throttle, 0.0, 1.0, "%.2f");
    flag_row("left shoulder", held(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER));
    flag_row("right shoulder", held(pad, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER));
    flag_row("d-pad up", held(pad, SDL_GAMEPAD_BUTTON_DPAD_UP));
    flag_row("d-pad down", held(pad, SDL_GAMEPAD_BUTTON_DPAD_DOWN));
    flag_row("Y (north)", held(pad, SDL_GAMEPAD_BUTTON_NORTH));
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Flight condition input",
                              ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_rows("flight")) {
    bar_row("R-STICK Y (up +)", -axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHTY), -1.0,
            1.0, "%+.2f");
    bar_row("R-STICK X (right +)", axis_unit(pad, SDL_GAMEPAD_AXIS_RIGHTX),
            -1.0, 1.0, "%+.2f");
    flag_row("d-pad right", held(pad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT));
    flag_row("d-pad left", held(pad, SDL_GAMEPAD_BUTTON_DPAD_LEFT));
    flag_row("back (reset)", held(pad, SDL_GAMEPAD_BUTTON_BACK));
    reading_row("-> altitude", input->altitude_m, "m", "%.0f");
    reading_row("-> airspeed", input->airspeed_ms, "m/s", "%.1f");
    reading_row("-> OAT offset", input->oat_offset_c, "degC", "%.1f");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Bindings", ImGuiTreeNodeFlags_DefaultOpen) &&
      ImGui::BeginTable("bindings", 3,
                        ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_BordersInnerV)) {
    ImGui::TableSetupColumn("Control", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 9.0f);
    ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Keys", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 6.0f);
    ImGui::TableHeadersRow();
    for (size_t i = 0; i < SDL_arraysize(BIND_ROWS); i++) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(BIND_ROWS[i][0]);
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(BIND_ROWS[i][1]);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextDisabled("%s", BIND_ROWS[i][2]);
    }
    ImGui::EndTable();
  }

  ImGui::End();
}
