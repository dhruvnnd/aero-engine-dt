#include "ui/torque_trace_panel.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imgui.h"
#include "implot.h"
#include "math/units.h"
#include "physics/engine_trace.h"
#include "ui/cyl_colors.h"
#include "ui/panel_names.h"

enum CylMode {
  MODE_NET = 0,
  MODE_GAS,
  MODE_INERTIA,
  MODE_PRESSURE,
  MODE_COUNT
};

static const char *MODE_LABEL[MODE_COUNT] = {"Net", "Gas", "Inertia",
                                             "Pressure"};
static const char *MODE_TITLE[MODE_COUNT] = {
    "Cylinder torque (gas + inertia)", "Cylinder gas torque",
    "Cylinder inertia torque", "Cylinder pressure"};
static const char *MODE_UNIT[MODE_COUNT] = {"N*m", "N*m", "N*m", "kPa"};

struct UiState {
  bool paused = false;
  bool auto_y = true;
  int mode = MODE_NET;
  bool snap_ready = false;
};

static UiState g;
static EngineTrace g_snap; /* frozen copy while paused */

/* The newest cycle, reordered by crank angle so it plots left to right over
 * 0-720 deg with no wrap-around line. Samples before the seam are last
 * cycle's, after it this cycle's. */
struct Sweep {
  int n;
  float x[ENGINE_TRACE_CAPACITY];
  float total[ENGINE_TRACE_CAPACITY];
  float rpm[ENGINE_TRACE_CAPACITY];
  float cyl[ENGINE_MAX_CYLINDERS][ENGINE_TRACE_CAPACITY];
  double now_deg; /* crank angle of the newest sample */
  double mean_nm; /* mean total torque over the sweep */
  double rpm_mean;
};
static Sweep g_sweep;

static float cyl_value(const EngineTraceSample *s, int c, int mode) {
  switch (mode) {
  case MODE_GAS:
    return s->cyl_gas_nm[c];
  case MODE_INERTIA:
    return s->cyl_inertia_nm[c];
  case MODE_PRESSURE:
    return s->cyl_pressure_kpa[c];
  default:
    return s->cyl_gas_nm[c] + s->cyl_inertia_nm[c];
  }
}

static void build_sweep(const EngineTrace *tr, int nc, int mode) {
  Sweep &w = g_sweep;
  w.n = engine_trace_last_cycle_count(tr);
  w.now_deg = 0.0;
  w.mean_nm = 0.0;
  w.rpm_mean = 0.0;
  if (w.n == 0) {
    return;
  }
  const int first = engine_trace_count(tr) - w.n;

  /* chronological index where the crank angle wraps 720 -> 0 (0 if it doesn't)
   */
  int seam = 0;
  for (int i = 1; i < w.n; i++) {
    if (engine_trace_at(tr, first + i)->theta_deg <
        engine_trace_at(tr, first + i - 1)->theta_deg) {
      seam = i;
      break;
    }
  }

  double sum_t = 0.0;
  double sum_rpm = 0.0;
  for (int k = 0; k < w.n; k++) {
    const EngineTraceSample *s = engine_trace_at(tr, first + (seam + k) % w.n);
    w.x[k] = s->theta_deg;
    w.total[k] = s->torque_nm;
    w.rpm[k] = (float)rad_s_to_rpm((double)s->omega_rad_s);
    for (int c = 0; c < nc; c++) {
      w.cyl[c][k] = cyl_value(s, c, mode);
    }
    sum_t += s->torque_nm;
    sum_rpm += w.rpm[k];
  }
  w.now_deg = engine_trace_at(tr, engine_trace_count(tr) - 1)->theta_deg;
  w.mean_nm = sum_t / w.n;
  w.rpm_mean = sum_rpm / w.n;
}

static void vline(const char *id, double x, ImVec4 color, float weight) {
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = weight;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
  ImPlot::PlotInfLines(id, &x, 1, spec);
}

static void hline(const char *id, double y, ImVec4 color) {
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = 1.0f;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit |
               ImPlotInfLinesFlags_Horizontal;
  ImPlot::PlotInfLines(id, &y, 1, spec);
}

/* Shared axis setup: crank angle 0-720 on x (ticked every 90 deg), y auto-fit
 * or left where the user put it. */
static void setup_axes(const char *y_label, bool bottom_row) {
  ImPlot::SetupAxes(bottom_row ? "crank angle (deg)" : NULL, y_label,
                    bottom_row ? ImPlotAxisFlags_None
                               : ImPlotAxisFlags_NoTickLabels,
                    g.auto_y ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None);
  ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 720.0, ImPlotCond_Once);
  ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0.0, 720.0);
  ImPlot::SetupAxisTicks(ImAxis_X1, 0.0, 720.0, 9);
}

static void plot_cylinders(int nc, const double *offset) {
  if (!ImPlot::BeginPlot(MODE_TITLE[g.mode], ImVec2(-1.0f, -1.0f),
                         ImPlotFlags_NoMouseText)) {
    return;
  }
  setup_axes(MODE_UNIT[g.mode], false);
  ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);

  /* each cylinder's firing TDC, in its own colour */
  for (int c = 0; c < nc; c++) {
    char id[16];
    snprintf(id, sizeof id, "##tdc%d", c);
    ImVec4 col = CYL_COLORS[c];
    col.w = 0.30f;
    vline(id, offset[c], col, 1.0f);
  }
  if (g.mode != MODE_PRESSURE) {
    hline("##zero", 0.0, ImVec4(1.0f, 1.0f, 1.0f, 0.25f));
  }

  for (int c = 0; c < nc; c++) {
    char label[16];
    snprintf(label, sizeof label, "Cyl %d", c + 1);
    ImPlotSpec line;
    line.LineColor = CYL_COLORS[c];
    line.LineWeight = 1.6f;
    ImPlot::PlotLine(label, g_sweep.x, g_sweep.cyl[c], g_sweep.n, line);
  }
  vline("##now", g_sweep.now_deg, ImVec4(1.0f, 1.0f, 1.0f, 0.45f), 1.0f);
  ImPlot::EndPlot();
}

static void plot_total(void) {
  if (!ImPlot::BeginPlot("Engine torque (all cylinders)", ImVec2(-1.0f, -1.0f),
                         ImPlotFlags_NoMouseText | ImPlotFlags_NoLegend)) {
    return;
  }
  setup_axes("N*m", false);
  hline("##zero", 0.0, ImVec4(1.0f, 1.0f, 1.0f, 0.25f));
  hline("##mean", g_sweep.mean_nm, ImVec4(1.00f, 0.75f, 0.20f, 0.85f));
  ImPlotSpec line;
  line.LineColor = ImVec4(0.92f, 0.92f, 0.92f, 1.0f);
  line.LineWeight = 1.8f;
  ImPlot::PlotLine("Total", g_sweep.x, g_sweep.total, g_sweep.n, line);
  vline("##now", g_sweep.now_deg, ImVec4(1.0f, 1.0f, 1.0f, 0.45f), 1.0f);
  ImPlot::EndPlot();
}

static void plot_speed(void) {
  if (!ImPlot::BeginPlot("Crank speed", ImVec2(-1.0f, -1.0f),
                         ImPlotFlags_NoMouseText | ImPlotFlags_NoLegend)) {
    return;
  }
  setup_axes("rpm", true);
  ImPlotSpec line;
  line.LineColor = ImVec4(0.55f, 0.85f, 0.90f, 1.0f);
  line.LineWeight = 1.6f;
  ImPlot::PlotLine("RPM", g_sweep.x, g_sweep.rpm, g_sweep.n, line);
  vline("##now", g_sweep.now_deg, ImVec4(1.0f, 1.0f, 1.0f, 0.45f), 1.0f);
  ImPlot::EndPlot();
}

void torque_trace_panel_draw(bool *open, const EngineTrace *trace,
                             const EngineConfig *cfg) {
  ImGui::SetNextWindowSize(ImVec2(640.0f, 700.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_TORQUE_TRACE, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused && trace) {
    memcpy(&g_snap, trace, sizeof g_snap);
    g.snap_ready = true;
  }
  ImGui::SameLine();
  ImGui::Checkbox("Auto Y", &g.auto_y);
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  for (int m = 0; m < MODE_COUNT; m++) {
    ImGui::SameLine();
    ImGui::RadioButton(MODE_LABEL[m], &g.mode, m);
  }

  const EngineTrace *src = (g.paused && g.snap_ready) ? &g_snap : trace;
  const int nc =
      cfg->num_cylinders < 1
          ? 1
          : (cfg->num_cylinders > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS
                                                       : cfg->num_cylinders);
  if (!src || engine_trace_count(src) < 2) {
    ImGui::TextDisabled("No samples yet -- start the engine (I).");
    ImGui::End();
    return;
  }

  build_sweep(src, nc, g.mode);
  double offset[ENGINE_MAX_CYLINDERS];
  engine_cylinder_phase_offsets(cfg, offset);

  ImGui::TextDisabled("%d samples over one cycle, %.2f deg/sample, "
                      "mean %.1f N*m, %.0f rpm",
                      g_sweep.n, 720.0 / (g_sweep.n > 0 ? g_sweep.n : 1),
                      g_sweep.mean_nm, g_sweep.rpm_mean);

  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  if (ImPlot::BeginSubplots("##torque_trace", 3, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    plot_cylinders(nc, offset);
    plot_total();
    plot_speed();
    ImPlot::EndSubplots();
  }
  ImPlot::PopStyleVar(2);

  ImGui::End();
}
