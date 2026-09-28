#include "ui/intake_panel.h"

#include "imgui.h"
#include "implot.h"
#include "ui/panel_names.h"

static const ImVec4 COL_IN(0.35f, 0.80f, 0.45f, 1.0f);
static const ImVec4 COL_OUT(0.85f, 0.45f, 0.75f, 1.0f);
static const ImVec4 COL_MAP(0.55f, 0.85f, 0.90f, 1.0f);
static const ImVec4 COL_TEXT(0.92f, 0.92f, 0.92f, 1.0f);

struct UiState {
  bool paused = false;
  bool snap_ready = false;
  IntakeTrends snap;
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

void intake_panel_draw(bool *open, const IntakeTrends *t,
                       const EngineConfig *cfg, double sample_period_s) {
  (void)cfg;
  ImGui::SetNextWindowSize(ImVec2(720.0f, 640.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_INTAKE, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    intake_trends_snapshot(&g.snap, t);
    g.snap_ready = true;
  }
  const IntakeTrends *src = (g.paused && g.snap_ready) ? &g.snap : t;
  const double dt = sample_period_s;
  const double max_span = (double)INTAKE_TRENDS_CAP * dt;

  const int n = history_count(&src->hist[INTM_MAP]);
  if (n > 0) {
    const double in_now = history_last(&src->hist[INTM_MDOT_IN]);
    const double out_now = history_last(&src->hist[INTM_MDOT_OUT]);
    ImGui::Text("map %.1f kPa", history_last(&src->hist[INTM_MAP]));
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::Text("in %.2f g/s", in_now);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::Text("out %.2f g/s", out_now);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    /* which way the plenum is moving right now */
    if (in_now > out_now + 0.05) {
      ImGui::TextColored(COL_IN, "filling");
    } else if (out_now > in_now + 0.05) {
      ImGui::TextColored(COL_OUT, "emptying");
    } else {
      ImGui::TextDisabled("steady");
    }
  }

  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  if (ImPlot::BeginSubplots("##intake_plots", 3, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    if (begin_time_plot("Manifold pressure", "kPa", max_span, dt, false)) {
      line("MAP", &src->hist[INTM_MAP], dt, COL_MAP, 2.2f);
      ImPlot::EndPlot();
    }
    if (begin_time_plot("Induction mass flow", "g/s", max_span, dt, false)) {
      line("throttle in", &src->hist[INTM_MDOT_IN], dt, COL_IN);
      line("cylinders out", &src->hist[INTM_MDOT_OUT], dt, COL_OUT);
      ImPlot::EndPlot();
    }
    if (begin_time_plot("Throttle command", "%", max_span, dt, true)) {
      line("throttle", &src->hist[INTM_THROTTLE], dt, COL_TEXT, 2.0f);
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
  ImPlot::PopStyleVar(2);

  ImGui::End();
}
