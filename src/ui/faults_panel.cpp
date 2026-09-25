#include "ui/faults_panel.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_MODIFIED(1.00f, 0.75f, 0.20f, 1.0f);

/* One editable trim. Ranges follow runs/fault_scenarios.md. */
struct Trim {
  const char *label;
  const char *short_name; /* for the summary and the log */
  double CylinderConfig::*field;
  double lo, hi, nominal;
  const char *fmt;
};

static const Trim TRIMS[] = {
    {"injector flow", "inj", &CylinderConfig::injector_flow_trim, 0.05, 2.0, 1.0,
     "%.2f"},
    {"compression", "comp", &CylinderConfig::compression_trim, 0.0, 1.2, 1.0,
     "%.2f"},
    {"spark offset (deg)", "spark", &CylinderConfig::spark_offset_deg, -35.0,
     35.0, 0.0, "%+.1f"},
    {"intake leak", "leak", &CylinderConfig::intake_leak_frac, 0.0, 1.0, 0.0,
     "%.2f"},
    {"cooling", "cool", &CylinderConfig::cooling_trim, 0.05, 1.5, 1.0, "%.2f"},
};
static const int TRIM_COUNT = (int)(sizeof TRIMS / sizeof TRIMS[0]);

struct Preset {
  const char *label;
  const char *description;
  int trim;
  double value;
};

/* Each sets one trim on the selected cylinder (values from the scenario
 * catalog: lean/rich lambda bands, ragged-edge and total-misfire thresholds). */
static const Preset PRESETS[] = {
    {"Compression loss", "compression 0.50", 1, 0.50},
    {"Spark retard", "spark offset +20 deg", 2, 20.0},
    {"Lean injector", "injector flow 0.35 (lambda ~2.9, misfire)", 0, 0.35},
    {"Rich injector", "injector flow 1.80", 0, 1.80},
    {"Intake leak", "intake leak 0.50", 3, 0.50},
    {"Cooling loss", "cooling 0.30", 4, 0.30},
};
static const int PRESET_COUNT = (int)(sizeof PRESETS / sizeof PRESETS[0]);

static bool differs(double v, double nominal) { return fabs(v - nominal) > 1e-6; }

static void log_trim(EventLog *log, double t, int cyl, const Trim &tr,
                     double value) {
  event_log_push(log, t, EVENT_INFO, "FAULT", "cyl %d %s = %.2f", cyl + 1,
                 tr.short_name, value);
}

static void reset_cylinder(CylinderConfig *c) {
  *c = cylinder_config_default();
}

static bool cylinder_modified(const CylinderConfig &c) {
  for (int i = 0; i < TRIM_COUNT; i++) {
    if (differs(c.*(TRIMS[i].field), TRIMS[i].nominal)) {
      return true;
    }
  }
  return false;
}

void faults_panel_draw(bool *open, CylinderConfig cfg[ENGINE_MAX_CYLINDERS],
                       int num_cyl, EventLog *log, double sim_time_s) {
  static int selected = 0;

  ImGui::SetNextWindowSize(ImVec2(420.0f, 460.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_FAULTS, open)) {
    ImGui::End();
    return;
  }

  const int nc = num_cyl < 1 ? 1
                             : (num_cyl > ENGINE_MAX_CYLINDERS
                                    ? ENGINE_MAX_CYLINDERS
                                    : num_cyl);
  if (selected >= nc) {
    selected = nc - 1;
  }

  ImGui::TextDisabled("cylinder");
  ImGui::SameLine();
  for (int i = 0; i < nc; i++) {
    if (i > 0) {
      ImGui::SameLine();
    }
    char label[16];
    snprintf(label, sizeof label, "%d", i + 1);
    const bool modified = cylinder_modified(cfg[i]);
    if (modified) {
      ImGui::PushStyleColor(ImGuiCol_Text, COL_MODIFIED);
    }
    if (ImGui::RadioButton(label, selected == i)) {
      selected = i;
    }
    if (modified) {
      ImGui::PopStyleColor();
    }
  }

  CylinderConfig &c = cfg[selected];
  ImGui::Separator();

  if (ImGui::BeginTable("trims", 3)) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 9.5f);
    ImGui::TableSetupColumn("slider", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("reset", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 2.0f);
    for (int i = 0; i < TRIM_COUNT; i++) {
      const Trim &tr = TRIMS[i];
      double &v = c.*(tr.field);
      const bool modified = differs(v, tr.nominal);

      ImGui::PushID(i);
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      if (modified) {
        ImGui::TextColored(COL_MODIFIED, "%s", tr.label);
      } else {
        ImGui::TextUnformatted(tr.label);
      }
      ImGui::TableSetColumnIndex(1);
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::SliderScalar("##v", ImGuiDataType_Double, &v, &tr.lo, &tr.hi,
                          tr.fmt);
      if (ImGui::IsItemDeactivatedAfterEdit()) {
        log_trim(log, sim_time_s, selected, tr, v);
      }
      ImGui::TableSetColumnIndex(2);
      ImGui::BeginDisabled(!modified);
      if (ImGui::Button("R")) {
        v = tr.nominal;
        log_trim(log, sim_time_s, selected, tr, v);
      }
      ImGui::EndDisabled();
      if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("reset to nominal (%.2f)", tr.nominal);
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::TextDisabled("Ctrl+click a slider to type a value");

  ImGui::Separator();
  ImGui::TextDisabled("presets for cylinder %d", selected + 1);
  for (int i = 0; i < PRESET_COUNT; i++) {
    const Preset &p = PRESETS[i];
    if (i % 2 != 0) {
      ImGui::SameLine();
    }
    if (ImGui::Button(p.label)) {
      c.*(TRIMS[p.trim].field) = p.value;
      event_log_push(log, sim_time_s, EVENT_INFO, "FAULT", "cyl %d %s",
                     selected + 1, p.description);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("%s", p.description);
    }
  }

  ImGui::Separator();
  if (ImGui::Button("Clear this cylinder")) {
    reset_cylinder(&c);
    event_log_push(log, sim_time_s, EVENT_INFO, "FAULT", "cyl %d faults cleared",
                   selected + 1);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear all")) {
    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
      reset_cylinder(&cfg[i]);
    }
    event_log_push(log, sim_time_s, EVENT_INFO, "FAULT", "all faults cleared");
  }

  ImGui::Separator();
  ImGui::TextDisabled("active faults");
  bool any = false;
  for (int i = 0; i < nc; i++) {
    char line[128];
    int len = snprintf(line, sizeof line, "cyl %d:", i + 1);
    bool has = false;
    for (int t = 0; t < TRIM_COUNT; t++) {
      const double v = cfg[i].*(TRIMS[t].field);
      if (differs(v, TRIMS[t].nominal) && len < (int)sizeof line) {
        len += snprintf(line + len, sizeof line - (size_t)len, " %s %.2f",
                        TRIMS[t].short_name, v);
        has = true;
      }
    }
    if (has) {
      ImGui::TextColored(COL_MODIFIED, "%s", line);
      any = true;
    }
  }
  if (!any) {
    ImGui::TextDisabled("none");
  }

  ImGui::End();
}
