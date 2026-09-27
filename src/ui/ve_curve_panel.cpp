#include "ui/ve_curve_panel.h"

#include "imgui.h"
#include "implot.h"
#include "physics/crank_thermo.h"
#include "ui/panel_names.h"

#define VE_CURVE_SAMPLES 200

static void vline(const char *id, double x, ImVec4 color) {
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = 1.5f;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
  ImPlot::PlotInfLines(id, &x, 1, spec);
}

void ve_curve_panel_draw(bool *open, const EngineConfig *cfg,
                         double current_rpm) {
  ImGui::SetNextWindowSize(ImVec2(560.0f, 400.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_VE_CURVE, open)) {
    ImGui::End();
    return;
  }

  const EngineGeometry &g = cfg->geom;
  /* Sweep from 0 to comfortably past where the curve floors on both sides,
   * so the shape (and the flat floor beyond it) is fully visible. */
  double hi = g.ve_peak_rpm + 1.6 * g.ve_width_rpm;
  if (current_rpm > hi) {
    hi = current_rpm * 1.15;
  }

  static float x[VE_CURVE_SAMPLES];
  static float y[VE_CURVE_SAMPLES];
  for (int i = 0; i < VE_CURVE_SAMPLES; i++) {
    double rpm = hi * (double)i / (double)(VE_CURVE_SAMPLES - 1);
    x[i] = (float)rpm;
    y[i] = (float)volumetric_efficiency(rpm, 0.0, &g);
  }

  ImGui::Text("Peak %.3f @ %.0f rpm", g.ve_peak, g.ve_peak_rpm);
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  ImGui::Text("Floor %.3f", g.ve_min);
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  ImGui::Text("Width %.0f rpm", g.ve_width_rpm);
  if (current_rpm > 0.0) {
    double now_ve = volumetric_efficiency(current_rpm, 0.0, &g);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.90f, 1.0f), "now %.3f @ %.0f rpm",
                       now_ve, current_rpm);
  }

  if (ImPlot::BeginPlot("##ve_curve", ImVec2(-1.0f, -1.0f),
                        ImPlotFlags_NoMouseText | ImPlotFlags_NoLegend)) {
    ImPlot::SetupAxes("rpm", "volumetric efficiency", ImPlotAxisFlags_None,
                      ImPlotAxisFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.2, ImPlotCond_Once);

    if (cfg->ecu_fitted && cfg->ecu.idle_target_rpm > 0.0) {
      vline("##idle", cfg->ecu.idle_target_rpm,
            ImVec4(0.60f, 0.90f, 0.60f, 0.55f));
    }
    vline("##peak", g.ve_peak_rpm, ImVec4(1.00f, 0.75f, 0.20f, 0.65f));
    if (current_rpm > 0.0) {
      vline("##now", current_rpm, ImVec4(0.55f, 0.85f, 0.90f, 0.85f));
    }

    ImPlotSpec line;
    line.LineColor = ImVec4(0.92f, 0.92f, 0.92f, 1.0f);
    line.LineWeight = 1.8f;
    ImPlot::PlotLine("VE", x, y, VE_CURVE_SAMPLES, line);

    ImPlot::EndPlot();
  }

  ImGui::End();
}
