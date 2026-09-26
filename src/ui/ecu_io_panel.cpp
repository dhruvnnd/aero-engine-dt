#include "ui/ecu_io_panel.h"

#include <float.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_OK(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

/* What each fault does, in a line. */
static const char *fault_hint(EcuFaultKind k) {
  switch (k) {
  case ECU_FAULT_OFFSET:
    return "The ECU reads the speed high or low, so it holds the wrong speed.";
  case ECU_FAULT_SCALE:
    return "The ECU reads the speed scaled: the same error grows with speed.";
  case ECU_FAULT_STUCK:
    return "The reading freezes: the ECU can't see a load step, so it won't "
           "correct it.";
  case ECU_FAULT_DROPOUT:
    return "The ECU reads 0: it sees a huge error and opens the throttle to "
           "its limit.";
  default:
    return "The ECU reads the true crank speed.";
  }
}

static void row(const char *signal, const char *value, const char *note,
                ImVec4 value_color = ImVec4(0.92f, 0.92f, 0.92f, 1.0f)) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(signal);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextColored(value_color, "%s", value);
  ImGui::TableSetColumnIndex(2);
  ImGui::TextDisabled("%s", note);
}

static bool begin_signal_table(const char *id) {
  if (!ImGui::BeginTable(id, 3, ImGuiTableFlags_RowBg)) {
    return false;
  }
  ImGui::TableSetupColumn("signal", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 11.0f);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 7.0f);
  ImGui::TableSetupColumn("note", ImGuiTableColumnFlags_WidthStretch);
  return true;
}

EcuIoActions ecu_io_panel_draw(bool *open, const ModelState *s,
                               const EcuSensorFault *rpm_fault) {
  EcuIoActions act = {};
  ImGui::SetNextWindowSize(ImVec2(440.0f, 480.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ECU_IO, open)) {
    ImGui::End();
    return act;
  }
  const EcuState &e = s->ecu;
  char buf[48];

  if (!e.fitted) {
    ImGui::TextDisabled("No ECU fitted.");
    ImGui::SeparatorText("Throttle");
    if (begin_signal_table("io_bare")) {
      snprintf(buf, sizeof buf, "%.1f %%", e.pilot_throttle * 100.0);
      row("pilot throttle", buf, "goes straight to the engine");
      ImGui::EndTable();
    }
    ImGui::End();
    return act;
  }

  const bool faulted = rpm_fault->kind != ECU_FAULT_NONE;

  ImGui::SeparatorText("Inputs");
  if (begin_signal_table("io_in")) {
    snprintf(buf, sizeof buf, "%.0f rpm", e.rpm_seen);
    if (faulted) {
      char note[64];
      snprintf(note, sizeof note, "FAULT (%s): true speed %.0f rpm",
               ecu_fault_kind_name(rpm_fault->kind), s->rpm);
      row("crank speed", buf, note, COL_CAUTION);
    } else {
      row("crank speed", buf, "sensor, reads true");
    }
    row("engine running", s->engine.run_state == ENGINE_RUNNING ? "yes" : "no",
        "");
    row("ignition", s->engine.ignition_on ? "on" : "off", "");
    snprintf(buf, sizeof buf, "%.1f %%", e.pilot_throttle * 100.0);
    row("pilot throttle", buf, "command");
    ImGui::EndTable();
  }

  ImGui::SeparatorText("Output");
  if (begin_signal_table("io_out")) {
    snprintf(buf, sizeof buf, "%.1f %%", e.throttle_cmd * 100.0);
    char note[64];
    snprintf(note, sizeof note, "throttle to engine = pilot + %.1f %% governor",
             e.throttle_cmd > e.pilot_throttle
                 ? (e.throttle_cmd - e.pilot_throttle) * 100.0
                 : 0.0);
    row("throttle command", buf, note, COL_OK);
    ImGui::EndTable();
  }

  ImGui::SeparatorText("Sensor fault: crank speed");
  int kind = (int)rpm_fault->kind;
  bool changed = false;
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0f);
  if (ImGui::BeginCombo("##kind", ecu_fault_kind_name(rpm_fault->kind))) {
    for (int k = 0; k < (int)ECU_FAULT_KIND_COUNT; k++) {
      const bool selected = k == kind;
      if (ImGui::Selectable(ecu_fault_kind_name((EcuFaultKind)k), selected)) {
        kind = k;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  double value = rpm_fault->value;
  if (kind == (int)ECU_FAULT_OFFSET || kind == (int)ECU_FAULT_SCALE) {
    if (changed && rpm_fault->kind != (EcuFaultKind)kind) {
      value = kind == (int)ECU_FAULT_OFFSET ? 100.0 : 0.85; /* a useful start */
    }
    const bool offset = kind == (int)ECU_FAULT_OFFSET;
    const double lo = offset ? -400.0 : 0.5;
    const double hi = offset ? 400.0 : 1.5;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderScalar("##value", ImGuiDataType_Double, &value, &lo, &hi,
                            offset ? "%+.0f rpm" : "x %.2f",
                            ImGuiSliderFlags_AlwaysClamp)) {
      changed = true;
    }
  }
  if (faulted) {
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      kind = (int)ECU_FAULT_NONE;
      changed = true;
    }
  }
  ImGui::PushTextWrapPos(0.0f);
  ImGui::TextColored(COL_DIM, "%s", fault_hint((EcuFaultKind)kind));
  ImGui::PopTextWrapPos();

  if (changed) {
    act.set_rpm_fault = true;
    act.kind = (EcuFaultKind)kind;
    act.value = value;
  }

  ImGui::End();
  return act;
}
