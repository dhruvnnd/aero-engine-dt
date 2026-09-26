#include "ui/engine_faults_panel.h"

#include <stdio.h>

#include "imgui.h"
#include "ui/cyl_colors.h"
#include "ui/panel_names.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

static ImVec4 status_color(ChannelStatus st) {
  if (st == CHANNEL_ALERT) {
    return COL_WARNING;
  }
  if (st == CHANNEL_WARN) {
    return COL_CAUTION;
  }
  return ImGui::GetStyle().Colors[ImGuiCol_Text];
}

static void cell(int col, ImVec4 color, const char *fmt, double v) {
  char buf[32];
  snprintf(buf, sizeof buf, fmt, v);
  ImGui::TableSetColumnIndex(col);
  ImGui::TextColored(color, "%s", buf);
}

void engine_faults_panel_draw(bool *open, const CylinderConfig *cfg,
                              int num_cyl, const ModelState *s,
                              const EngineTrace *trace,
                              const EngineFaultTracker *tracker, double now_s) {
  ImGui::SetNextWindowSize(ImVec2(760.0f, 280.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ENGINE_FAULTS, open)) {
    ImGui::End();
    return;
  }
  const int nc = num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS : num_cyl;

  int faulty = 0;
  int flagged = 0;
  for (int i = 0; i < nc; i++) {
    faulty += tracker->cyl[i].faulty;
    flagged += tracker->cyl[i].faulty && tracker->cyl[i].now != CHANNEL_OK;
  }
  if (faulty == 0) {
    ImGui::TextDisabled("No faults injected (Fault Injection panel).");
    ImGui::End();
    return;
  }
  ImGui::Text("%d faulty cylinder%s, %d flagged by the monitors", faulty,
              faulty == 1 ? "" : "s", flagged);

  if (ImGui::BeginTable("engine_faults", 8,
                        ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
    ImGui::TableSetupColumn("cyl", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 2.5f);
    ImGui::TableSetupColumn("injected", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("CHT vs others", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 6.5f);
    ImGui::TableSetupColumn("EGT vs others", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 6.5f);
    ImGui::TableSetupColumn("lambda", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 3.5f);
    ImGui::TableSetupColumn("misfire", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 4.0f);
    ImGui::TableSetupColumn("power", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 4.0f);
    ImGui::TableSetupColumn("monitors", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

    for (int i = 0; i < nc; i++) {
      const CylFaultTrack &k = tracker->cyl[i];
      if (!k.faulty) {
        continue;
      }
      const CylSymptoms sy = engine_faults_symptoms(s, i, nc, trace);
      const CylinderState &c = s->cyl[i];

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(CYL_COLORS[i], "%d", i + 1);

      char text[96];
      cylinder_fault_text(&cfg[i], text, sizeof text);
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(text);

      cell(2, status_color(channel_status_for(c.cht_c, CHANNEL_RANGE_CHT)),
           "%+.0f degC", sy.d_cht_c);
      cell(3, status_color(channel_status_for(c.egt_c, CHANNEL_RANGE_EGT)),
           "%+.0f degC", sy.d_egt_c);
      cell(4, status_color(channel_status_for(c.lambda, CHANNEL_RANGE_LAMBDA)),
           "%.2f", sy.lambda);
      cell(5, c.misfire_rate > 0.5 ? COL_WARNING
              : c.misfire_rate > 0.0 ? COL_CAUTION
                                     : ImGui::GetStyle().Colors[ImGuiCol_Text],
           "%.0f %%", sy.misfire_pct);
      if (sy.power_pct < 0.0) {
        ImGui::TableSetColumnIndex(6);
        ImGui::TextDisabled("--");
      } else {
        cell(6, sy.power_pct < 90.0 ? COL_CAUTION
                                    : ImGui::GetStyle().Colors[ImGuiCol_Text],
             "%.0f %%", sy.power_pct);
      }

      ImGui::TableSetColumnIndex(7);
      if (k.now != CHANNEL_OK) {
        ImGui::TextColored(status_color(k.now), "%s after %.1f s",
                           k.now == CHANNEL_ALERT ? "WARNING" : "CAUTION",
                           k.first_flag_s - k.since_s);
      } else if (k.flagged) {
        ImGui::TextDisabled("flagged after %.1f s, clear now",
                            k.first_flag_s - k.since_s);
      } else {
        ImGui::TextColored(COL_CAUTION, "not flagged (%.0f s)", now_s - k.since_s);
      }
    }
    ImGui::EndTable();
  }

  ImGui::End();
}
