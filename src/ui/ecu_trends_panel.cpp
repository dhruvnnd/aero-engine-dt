#include "ui/ecu_trends_panel.h"

#include "imgui.h"
#include "implot.h"
#include "ui/panel_names.h"

static const ImVec4 COL_OK(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_INFO(0.35f, 0.65f, 0.95f, 1.0f);
static const ImVec4 COL_TEXT(0.92f, 0.92f, 0.92f, 1.0f);

struct UiState {
  bool paused = false;
  bool snap_ready = false;
  EcuTrends snap;
};
static UiState g;

struct SeriesRef {
  const History *hist;
  double dt;
};

/* x = seconds relative to the newest sample (<= 0), y = the stored value. */
static ImPlotPoint series_point(int idx, void *user) {
  const SeriesRef *ref = (const SeriesRef *)user;
  const int n = history_count(ref->hist);
  return ImPlotPoint((double)(idx - (n - 1)) * ref->dt,
                     history_at(ref->hist, idx));
}

static void line(const char *label, const History *h, double dt, ImVec4 color,
                 float weight = 1.6f) {
  const int n = history_count(h);
  if (n == 0) {
    return;
  }
  SeriesRef ref = {h, dt};
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = weight;
  ImPlot::PlotLineG(label, series_point, &ref, n, spec);
}

static void hline(const char *id, double y, ImVec4 color) {
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = 1.0f;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit |
               ImPlotInfLinesFlags_Horizontal;
  ImPlot::PlotInfLines(id, &y, 1, spec);
}

static bool begin_time_plot(const char *title, const char *y_label,
                            double max_span, double dt, bool bottom) {
  if (!ImPlot::BeginPlot(title, ImVec2(-1.0f, -1.0f), ImPlotFlags_NoMouseText)) {
    return false;
  }
  ImPlot::SetupAxes(bottom ? "seconds" : NULL, y_label,
                    bottom ? ImPlotAxisFlags_None : ImPlotAxisFlags_NoTickLabels,
                    ImPlotAxisFlags_AutoFit);
  ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
  ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, -max_span - dt, dt);
  ImPlot::SetupAxisLimits(ImAxis_X1, -max_span, 0.0, ImPlotCond_Once);
  return true;
}

void ecu_trends_panel_draw(bool *open, const EcuTrends *t,
                           const EngineConfig *cfg, double sample_period_s) {
  ImGui::SetNextWindowSize(ImVec2(720.0f, 720.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ECU_TRENDS, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    ecu_trends_snapshot(&g.snap, t);
    g.snap_ready = true;
  }
  const EcuTrends *src = (g.paused && g.snap_ready) ? &g.snap : t;
  const double dt = sample_period_s;
  const double max_span = (double)ECU_TRENDS_CAP * dt;

  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  if (ImPlot::BeginSubplots("##ecu_plots", 3, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    if (begin_time_plot("Crank speed vs idle target", "rpm", max_span, dt,
                        false)) {
      const History *target = &src->hist[ECUM_TARGET];
      const int n = history_count(target);
      if (n > 0 && history_at(target, n - 1) > 0.0) {
        hline("##target", history_at(target, n - 1),
              ImVec4(1.0f, 0.75f, 0.20f, 0.85f));
      }
      line("rpm", &src->hist[ECUM_RPM], dt, ImVec4(0.55f, 0.85f, 0.90f, 1.0f));
      ImPlot::EndPlot();
    }
    if (begin_time_plot("Throttle", "% throttle", max_span, dt, false)) {
      line("pilot", &src->hist[ECUM_PILOT], dt, COL_INFO);
      line("governor", &src->hist[ECUM_GOVERNOR], dt, COL_OK);
      line("command", &src->hist[ECUM_COMMAND], dt, COL_TEXT, 2.0f);
      ImPlot::EndPlot();
    }
    if (begin_time_plot("Governor terms", "% throttle", max_span, dt, true)) {
      hline("##limit", cfg->idle_max_throttle * 100.0,
            ImVec4(1.0f, 0.35f, 0.30f, 0.85f));
      hline("##zero", 0.0, ImVec4(1.0f, 1.0f, 1.0f, 0.25f));
      line("P", &src->hist[ECUM_P_TERM], dt, COL_CAUTION);
      line("I", &src->hist[ECUM_I_TERM], dt, ImVec4(0.85f, 0.45f, 0.75f, 1.0f));
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
  ImPlot::PopStyleVar(2);

  ImGui::End();
}
