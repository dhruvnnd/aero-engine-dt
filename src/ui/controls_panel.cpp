#include "ui/controls_panel.h"

#include <float.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);

static const char *run_state_name(EngineRunState st) {
  switch (st) {
  case ENGINE_CRANKING:
    return "CRANKING";
  case ENGINE_RUNNING:
    return "RUNNING";
  case ENGINE_STOPPED:
  default:
    return "STOPPED";
  }
}

/* One labelled slider row; `fmt` shows the live value on the slider. */
static void slider_row(const char *label, double *v, double lo, double hi,
                       const char *fmt) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);
  ImGui::TableSetColumnIndex(1);
  ImGui::SetNextItemWidth(-FLT_MIN);
  ImGui::PushID(label);
  ImGui::SliderScalar("##v", ImGuiDataType_Double, v, &lo, &hi, fmt,
                      ImGuiSliderFlags_AlwaysClamp);
  ImGui::PopID();
}

ControlActions controls_panel_draw(bool *open, SdlInputState *input,
                                   const ModelState *s, int *sensor_mode) {
  ControlActions act = {};

  ImGui::SetNextWindowSize(ImVec2(400.0f, 360.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_CONTROLS, open)) {
    ImGui::End();
    return act;
  }

  const EngineRunState rs = s->engine.run_state;
  ImGui::TextDisabled("engine");
  ImGui::SameLine();
  if (rs == ENGINE_CRANKING) {
    ImGui::TextColored(COL_CAUTION, "%s", run_state_name(rs));
  } else if (rs == ENGINE_STOPPED) {
    ImGui::TextDisabled("%s", run_state_name(rs));
  } else {
    ImGui::TextUnformatted(run_state_name(rs));
  }

  ImGui::BeginDisabled(rs != ENGINE_STOPPED);
  if (ImGui::Button("Start  (I)")) {
    act.start_engine = true;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(rs == ENGINE_STOPPED);
  if (ImGui::Button("Stop  (O)")) {
    act.stop_engine = true;
  }
  ImGui::EndDisabled();

  const EcuState &ecu = s->ecu;
  const bool governor_available =
      ecu.fitted && ecu.idle_mode != ECU_IDLE_DISABLED;
  ImGui::BeginDisabled(!governor_available);
  bool governor_on = ecu.idle_enabled != 0;
  if (ImGui::Checkbox("Idle governor  (K)", &governor_on)) {
    act.toggle_idle_governor = true;
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(!ecu.fitted
                          ? "No ECU fitted (ecu_fitted = 0)."
                      : ecu.idle_mode == ECU_IDLE_DISABLED
                          ? "No governor configured (idle_target_rpm = 0)."
                          : "The ECU holds idle speed by adding throttle.\n"
                            "Off: idle falls to the engine's natural speed.\n"
                            "See the ECU panel for the loop.");
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%s",
                      ecu.fitted ? ecu_idle_mode_name(ecu.idle_mode) : "NO ECU");

  ImGui::Separator();
  if (ImGui::BeginTable("sliders", 2)) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 7.0f);
    ImGui::TableSetupColumn("slider", ImGuiTableColumnFlags_WidthStretch);

    double throttle_pct = input->throttle * 100.0;
    slider_row("throttle", &throttle_pct, 0.0, 100.0, "%.0f %%");
    input->throttle = throttle_pct / 100.0;

    slider_row("altitude", &input->altitude_m, 0.0, SDL_INPUT_ALT_MAX_M,
               "%.0f m");
    slider_row("airspeed", &input->airspeed_ms, 0.0, SDL_INPUT_SPD_MAX_MS,
               "%.1f m/s");
    slider_row("OAT offset", &input->oat_offset_c, SDL_INPUT_OAT_MIN_C,
               SDL_INPUT_OAT_MAX_C, "%+.1f degC");
    ImGui::EndTable();
  }
  ImGui::TextDisabled("Ctrl+click a slider to type a value");

  if (ImGui::Button("Reset flight condition  (R)")) {
    input->altitude_m = 0.0;
    input->airspeed_ms = 0.0;
    input->oat_offset_c = 0.0;
  }

  ImGui::Separator();
  ImGui::TextDisabled("display feed");
  ImGui::SameLine();
  if (ImGui::RadioButton("Model (exact)", *sensor_mode == 0)) {
    *sensor_mode = 0;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Sensor (noisy)", *sensor_mode != 0)) {
    *sensor_mode = 1;
  }

  ImGui::End();
  return act;
}
