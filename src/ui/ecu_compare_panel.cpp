#include "ui/ecu_compare_panel.h"

#include <stdio.h>

#include "imgui.h"
#include "implot.h"
#include "ui/panel_names.h"

static const ImVec4 COL_ECU(0.35f, 0.80f, 0.45f, 1.0f);  /* with the ECU */
static const ImVec4 COL_BARE(1.00f, 0.75f, 0.20f, 1.0f); /* without it */
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 COL_DIM(0.60f, 0.62f, 0.62f, 1.0f);

struct UiState {
  bool paused = false;
  bool snap_ready = false;
  EcuCompareTrends snap;
};
static UiState g;

static const char *run_state_name(EngineRunState st) {
  switch (st) {
  case ENGINE_CRANKING:
    return "cranking";
  case ENGINE_RUNNING:
    return "running";
  default:
    return "STOPPED";
  }
}

/* ---- summary + table --------------------------------------------------- */

static void draw_summary(const ModelState *with_ecu,
                         const ModelState *without_ecu) {
  const EcuCompareSummary c = ecu_compare_summarize(with_ecu, without_ecu);
  ImGui::PushTextWrapPos(0.0f);
  if (c.bare_stalled) {
    ImGui::TextColored(COL_WARNING,
                       "Without the ECU this engine would have stalled.");
  } else if (c.compensating) {
    ImGui::TextColored(COL_ECU,
                       "The ECU is adding %.1f %% throttle, holding the engine "
                       "%.0f rpm above where it would sit on its own.",
                       c.extra_throttle_pct, c.rpm_gain);
  } else {
    ImGui::TextColored(COL_DIM,
                       "The ECU isn't adding anything: both engines behave the "
                       "same.");
  }
  ImGui::PopTextWrapPos();
}

static void row(const char *label, const char *a, const char *b) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextColored(COL_ECU, "%s", a);
  ImGui::TableSetColumnIndex(2);
  ImGui::TextColored(COL_BARE, "%s", b);
}

static void draw_table(const ModelState *with_ecu,
                       const ModelState *without_ecu) {
  if (!ImGui::BeginTable("cmp", 3, ImGuiTableFlags_RowBg)) {
    return;
  }
  ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 10.0f);
  ImGui::TableSetupColumn("with ECU", ImGuiTableColumnFlags_WidthStretch);
  ImGui::TableSetupColumn("without ECU", ImGuiTableColumnFlags_WidthStretch);
  ImGui::TableHeadersRow();

  char a[32], b[32];
  snprintf(a, sizeof a, "%.0f rpm", with_ecu->rpm);
  snprintf(b, sizeof b, "%.0f rpm", without_ecu->rpm);
  row("crank speed", a, b);
  snprintf(a, sizeof a, "%.1f %%", with_ecu->ecu.throttle_cmd * 100.0);
  snprintf(b, sizeof b, "%.1f %%", without_ecu->ecu.throttle_cmd * 100.0);
  row("throttle", a, b);
  snprintf(a, sizeof a, "%.1f kPa", with_ecu->engine.map_kpa);
  snprintf(b, sizeof b, "%.1f kPa", without_ecu->engine.map_kpa);
  row("manifold pressure", a, b);
  row("engine", run_state_name(with_ecu->engine.run_state),
      run_state_name(without_ecu->engine.run_state));
  ImGui::EndTable();
}

/* ---- plots ------------------------------------------------------------- */

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

static void line(const char *label, const History *h, double dt, ImVec4 color) {
  const int n = history_count(h);
  if (n == 0) {
    return;
  }
  SeriesRef ref = {h, dt};
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = 1.8f;
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

static void draw_plots(const EcuCompareTrends *t, double dt) {
  const double max_span = (double)ECU_COMPARE_CAP * dt;
  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  if (ImPlot::BeginSubplots("##ecu_cmp_plots", 2, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    if (begin_time_plot("Crank speed", "rpm", max_span, dt, false)) {
      line("with ECU", &t->hist[ECUC_RPM_ECU], dt, COL_ECU);
      line("without ECU", &t->hist[ECUC_RPM_BARE], dt, COL_BARE);
      ImPlot::EndPlot();
    }
    if (begin_time_plot("Throttle at the engine", "% throttle", max_span, dt,
                        true)) {
      line("with ECU", &t->hist[ECUC_THR_ECU], dt, COL_ECU);
      line("without ECU", &t->hist[ECUC_THR_BARE], dt, COL_BARE);
      ImPlot::EndPlot();
    }
    ImPlot::EndSubplots();
  }
  ImPlot::PopStyleVar(2);
}

/* ---- panel ------------------------------------------------------------- */

bool ecu_compare_panel_draw(bool *open, const ModelState *with_ecu,
                            const ModelState *without_ecu,
                            const EcuCompareTrends *t, bool fitted,
                            double sample_period_s) {
  bool resync = false;
  ImGui::SetNextWindowSize(ImVec2(640.0f, 720.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ECU_COMPARE, open)) {
    ImGui::End();
    return resync;
  }
  if (!fitted) {
    ImGui::TextDisabled("No ECU fitted: nothing to compare.");
    ImGui::End();
    return resync;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    ecu_compare_snapshot(&g.snap, t);
    g.snap_ready = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Resync")) {
    resync = true;
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Copy the ECU engine's state into the one without an ECU\n"
                      "(for instance after it has stalled).");
  }

  draw_summary(with_ecu, without_ecu);
  ImGui::Separator();
  draw_table(with_ecu, without_ecu);
  ImGui::Separator();
  draw_plots((g.paused && g.snap_ready) ? &g.snap : t, sample_period_s);

  ImGui::End();
  return resync;
}
