#include "ui/ecu_faults_panel.h"

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_OK(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

EcuFaultsActions ecu_faults_panel_draw(bool *open, const ModelState *s) {
  EcuFaultsActions act = {};
  ImGui::SetNextWindowSize(ImVec2(680.0f, 400.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ECU_FAULTS, open)) {
    ImGui::End();
    return act;
  }
  const EcuState &e = s->ecu;
  if (!e.fitted) {
    ImGui::TextDisabled("No ECU fitted.");
    ImGui::End();
    return act;
  }

  bool on = e.diag_enabled != 0;
  if (ImGui::Checkbox("Diagnostics", &on)) {
    act.toggle_diagnostics = true;
  }
  ImGui::SameLine();
  if (!e.diag_enabled) {
    ImGui::TextColored(COL_CAUTION, "[ OFF ]");
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextColored(COL_DIM,
                       "The ECU trusts the crank sensor without checking it, "
                       "and records nothing.");
    ImGui::PopTextWrapPos();
  } else if (e.speed_source == ECU_SRC_NONE) {
    ImGui::TextColored(COL_WARNING, "[ LIMP HOME ]");
    ImGui::TextColored(COL_DIM,
                       "No trusted speed sensor: fixed idle throttle.");
  } else if (e.speed_source == ECU_SRC_SECONDARY) {
    ImGui::TextColored(COL_CAUTION, "[ DEGRADED ]");
    ImGui::TextColored(COL_DIM,
                       "Crank sensor not trusted: using the alternator.");
  } else if (ecu_diag_active_count(&e.diag) > 0) {
    ImGui::TextColored(COL_CAUTION, "[ FAULT ]");
    ImGui::TextColored(COL_DIM, "Using the crank sensor. A fault is present.");
  } else {
    ImGui::TextColored(COL_OK, "[ HEALTHY ]");
    ImGui::TextColored(COL_DIM, "Using the crank sensor.");
  }

  const int latched = ecu_diag_latched_count(&e.diag);
  ImGui::SeparatorText("Fault codes");
  if (latched == 0) {
    ImGui::TextDisabled("No fault codes.");
  } else if (ImGui::BeginTable("dtcs", 6,
                               ImGuiTableFlags_RowBg |
                                   ImGuiTableFlags_Resizable)) {
    ImGui::TableSetupColumn("code", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 3.5f);
    ImGui::TableSetupColumn("description", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("status", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 4.5f);
    ImGui::TableSetupColumn("count", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 3.0f);
    ImGui::TableSetupColumn("first", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 4.5f);
    ImGui::TableSetupColumn("freeze frame", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

    for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
      const EcuDtc &d = e.diag.dtc[i];
      if (!d.latched && !d.active) {
        continue;
      }
      const EcuDtcInfo *info = ecu_dtc_info((EcuDtcId)i);
      const ImVec4 col =
          d.active ? (info->severity ? COL_WARNING : COL_CAUTION) : COL_DIM;
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(col, "%s", info->code);
      ImGui::TableSetColumnIndex(1);
      ImGui::TextColored(col, "%s", info->description);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextColored(col, "%s", d.active ? "ACTIVE" : "latched");
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%d", d.count);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.1f s", d.first_s);
      ImGui::TableSetColumnIndex(5);
      ImGui::TextDisabled("crank %.0f  alt %.0f  pilot %.0f %%  using %s",
                          d.freeze.rpm1, d.freeze.rpm2,
                          d.freeze.pilot_throttle * 100.0,
                          ecu_speed_source_name(d.freeze.source));
    }
    ImGui::EndTable();
  }

  ImGui::BeginDisabled(latched == 0);
  if (ImGui::Button("Clear codes")) {
    act.clear_codes = true;
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Forget the recorded codes and their freeze frames.\n"
                      "A fault that is still present registers again.");
  }

  ImGui::End();
  return act;
}
