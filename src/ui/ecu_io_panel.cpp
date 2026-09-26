#include "ui/ecu_io_panel.h"

#include <float.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_OK(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

static const char *CHANNEL_NAME[2] = {"crank speed", "alternator speed"};

static const char *fault_hint(EcuFaultKind k) {
  switch (k) {
  case ECU_FAULT_OFFSET:
    return "Reads the speed high or low.";
  case ECU_FAULT_SCALE:
    return "Reads the speed scaled: the error grows with speed.";
  case ECU_FAULT_STUCK:
    return "The reading freezes.";
  case ECU_FAULT_DROPOUT:
    return "Reads 0.";
  default:
    return "Reads the true crank speed.";
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

/* One speed sensor's row: what the ECU reads, and whether it is being used. */
static void speed_row(int channel, double reads, double truth,
                      const EcuSensorFault &fault, bool in_use) {
  char value[32];
  char note[96];
  snprintf(value, sizeof value, "%.0f rpm", reads);
  const bool faulted = fault.kind != ECU_FAULT_NONE;
  if (faulted) {
    snprintf(note, sizeof note, "FAULT (%s): true %.0f rpm%s",
             ecu_fault_kind_name(fault.kind), truth,
             in_use ? "" : "  [not used]");
  } else {
    snprintf(note, sizeof note, "%s", in_use ? "in use" : "not used");
  }
  row(CHANNEL_NAME[channel], value, note, faulted ? COL_CAUTION : COL_OK);
}

/* Fault controls for one speed sensor. */
static void fault_row(int channel, const EcuSensorFault &f, EcuIoActions *act) {
  ImGui::PushID(channel);
  ImGui::TextUnformatted(CHANNEL_NAME[channel]);
  int kind = (int)f.kind;
  bool changed = false;
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
  if (ImGui::BeginCombo("##kind", ecu_fault_kind_name(f.kind))) {
    for (int k = 0; k < (int)ECU_FAULT_KIND_COUNT; k++) {
      if (ImGui::Selectable(ecu_fault_kind_name((EcuFaultKind)k), k == kind)) {
        kind = k;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  double value = f.value;
  if (kind == (int)ECU_FAULT_OFFSET || kind == (int)ECU_FAULT_SCALE) {
    if (changed && f.kind != (EcuFaultKind)kind) {
      value = kind == (int)ECU_FAULT_OFFSET ? 100.0 : 0.85; /* a useful start */
    }
    const bool offset = kind == (int)ECU_FAULT_OFFSET;
    const double lo = offset ? -400.0 : 0.5;
    const double hi = offset ? 400.0 : 1.5;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0f);
    if (ImGui::SliderScalar("##value", ImGuiDataType_Double, &value, &lo, &hi,
                            offset ? "%+.0f rpm" : "x %.2f",
                            ImGuiSliderFlags_AlwaysClamp)) {
      changed = true;
    }
  }
  if (f.kind != ECU_FAULT_NONE) {
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      kind = (int)ECU_FAULT_NONE;
      changed = true;
    }
  }
  ImGui::TextColored(COL_DIM, "%s", fault_hint((EcuFaultKind)kind));
  if (changed) {
    act->set_fault = true;
    act->channel = channel;
    act->kind = (EcuFaultKind)kind;
    act->value = value;
  }
  ImGui::PopID();
}

EcuIoActions ecu_io_panel_draw(bool *open, const ModelState *s,
                               const EcuSensorFault faults[2]) {
  EcuIoActions act = {};
  ImGui::SetNextWindowSize(ImVec2(460.0f, 560.0f), ImGuiCond_FirstUseEver);
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

  ImGui::SeparatorText("Inputs");
  if (begin_signal_table("io_in")) {
    speed_row(0, e.rpm1_seen, s->rpm, faults[0],
              e.speed_source == ECU_SRC_PRIMARY);
    speed_row(1, e.rpm2_seen, s->rpm, faults[1],
              e.speed_source == ECU_SRC_SECONDARY);
    const bool degraded = e.diag_enabled && e.speed_source != ECU_SRC_PRIMARY;
    row("speed used", ecu_speed_source_name(e.speed_source),
        !e.diag_enabled ? "diagnostics off: trusts the crank sensor"
        : e.speed_source == ECU_SRC_NONE ? "no trusted speed: limp-home"
        : degraded                       ? "crank sensor not trusted"
                                         : "",
        e.speed_source == ECU_SRC_NONE ? COL_WARNING
        : degraded                     ? COL_CAUTION
                                       : COL_OK);
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

  ImGui::SeparatorText("Sensor faults");
  for (int c = 0; c < 2; c++) {
    fault_row(c, faults[c], &act);
  }

  ImGui::End();
  return act;
}
