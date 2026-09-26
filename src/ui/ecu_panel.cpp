#include "ui/ecu_panel.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_OK(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 COL_INFO(0.35f, 0.65f, 0.95f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

/* An engine within this many rpm of the target counts as "holding" it. */
static const double IN_BAND_RPM = 25.0;

static bool holding(const EcuState &e) {
  return e.idle_mode == ECU_IDLE_ACTIVE && fabs(e.idle_error_rpm) <= IN_BAND_RPM;
}

static ImVec4 mode_color(const EcuState &e) {
  switch (e.idle_mode) {
  case ECU_IDLE_ACTIVE:
    return holding(e) ? COL_OK : COL_INFO;
  case ECU_IDLE_LIMITED:
    return COL_WARNING;
  case ECU_IDLE_OFF:
    return COL_CAUTION;
  case ECU_IDLE_PILOT:
    return COL_INFO;
  default:
    return COL_DIM;
  }
}

static const char *status_text(const EcuState &e) {
  if (e.idle_mode == ECU_IDLE_ACTIVE) {
    return holding(e) ? "HOLDING TARGET" : "REGULATING";
  }
  return ecu_idle_mode_name(e.idle_mode);
}

/* One line saying what the governor is doing right now. */
static const char *explanation(const EcuState &e) {
  switch (e.idle_mode) {
  case ECU_IDLE_DISABLED:
    return "No governor configured (idle_target_rpm is 0 in the engine spec).";
  case ECU_IDLE_OFF:
    return "Switched off: idle falls to the engine's natural speed.";
  case ECU_IDLE_STANDBY:
    return "Engine not running: the governor is standing by.";
  case ECU_IDLE_PILOT:
    return "Pilot throttle is above its authority: the governor has stepped "
           "aside.";
  case ECU_IDLE_LIMITED:
    return "At full authority and still below the target: the load is more "
           "than it can carry.";
  case ECU_IDLE_ACTIVE:
    return holding(e) ? "The throttle it adds balances the load."
                      : "Moving the throttle to bring the speed back to the "
                        "target.";
  }
  return "";
}

static void row(const char *label, const char *unit, const char *fmt, double v) {
  char buf[48];
  snprintf(buf, sizeof buf, fmt, v);
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextUnformatted(buf);
  ImGui::TableSetColumnIndex(2);
  ImGui::TextDisabled("%s", unit);
}

EcuActions ecu_panel_draw(bool *open, const ModelState *s,
                          const EngineConfig *cfg) {
  EcuActions act = {};
  ImGui::SetNextWindowSize(ImVec2(400.0f, 440.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ECU, open)) {
    ImGui::End();
    return act;
  }
  const EcuState &e = s->ecu;

  if (!e.fitted) {
    ImGui::TextDisabled("No ECU fitted.");
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextColored(COL_DIM,
                       "The pilot's throttle drives the engine directly. Set "
                       "ecu_fitted = 1 in the engine spec to fit one.");
    ImGui::PopTextWrapPos();
    ImGui::End();
    return act;
  }

  ImGui::TextDisabled("idle governor");
  ImGui::SameLine();
  ImGui::TextColored(mode_color(e), "[ %s ]", status_text(e));

  ImGui::BeginDisabled(e.idle_mode == ECU_IDLE_DISABLED);
  bool on = e.idle_enabled != 0;
  if (ImGui::Checkbox("Enabled  (K)", &on)) {
    act.toggle_idle_governor = true;
  }
  ImGui::EndDisabled();

  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextColored(COL_DIM, "%s", explanation(e));
  ImGui::PopTextWrapPos();

  ImGui::Separator();
  if (ImGui::BeginTable("ecu_values", 3, ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 11.0f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 5.0f);
    ImGui::TableSetupColumn("unit", ImGuiTableColumnFlags_WidthStretch);
    row("target", "rpm", "%.0f", e.idle_target_rpm);
    row("ECU reads", "rpm", "%.0f", e.rpm_seen);
    if (fabs(e.rpm_seen - s->rpm) > 0.5) { /* a sensor fault: the ECU is misled */
      row("true speed", "rpm", "%.0f", s->rpm);
    }
    row("error", "rpm", "%+.0f", e.idle_error_rpm);
    row("P term", "% throttle", "%+.1f", e.idle_p_term * 100.0);
    row("I term", "% throttle", "%.1f", e.idle_i_term * 100.0);
    row("governor output", "% throttle", "%.1f", e.idle_throttle * 100.0);
    row("pilot throttle", "% throttle", "%.1f", e.pilot_throttle * 100.0);
    row("throttle command", "% throttle", "%.1f", e.throttle_cmd * 100.0);
    ImGui::EndTable();
  }

  /* How much of the governor's range is spent. A rising baseline at the same
   * conditions means the engine needs more help to idle: the ECU is
   * compensating for something. */
  const float used = cfg->ecu.idle_max_throttle > 0.0
                         ? (float)(e.idle_throttle / cfg->ecu.idle_max_throttle)
                         : 0.0f;
  char overlay[48];
  snprintf(overlay, sizeof overlay, "authority used %.0f %%", used * 100.0f);
  const bool saturated = e.idle_mode == ECU_IDLE_LIMITED;
  if (saturated) {
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, COL_WARNING);
  }
  ImGui::ProgressBar(used, ImVec2(-FLT_MIN, 0.0f), overlay);
  if (saturated) {
    ImGui::PopStyleColor();
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("How much of the governor's throttle authority is in use.\n"
                      "A rising baseline at the same conditions means the engine\n"
                      "needs more help to idle: the ECU is compensating for it.");
  }

  ImGui::End();
  return act;
}
