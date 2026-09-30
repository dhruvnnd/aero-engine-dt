#include "ui/pv_diagram_panel.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imgui.h"
#include "implot.h"
#include "telemetry/pv_diagram.h"
#include "ui/cyl_colors.h"
#include "ui/panel_names.h"

#define MAX_OVERLAY_CYCLES 4

struct UiState {
  bool paused = false;
  bool snap_ready = false;
  bool sel[ENGINE_MAX_CYLINDERS] = {true, true, true, true, true, true};
  int cycles = 1;
  bool auto_fit = true;
  bool log_axes = false;
  bool show_ref = true;
  bool show_markers = true;
};

static UiState g;
static EngineTrace g_snap; /* frozen copy while paused */
static PvLoop g_loop;      /* scratch: rebuilt and plotted one loop at a time */
static PvStats g_stats[ENGINE_MAX_CYLINDERS];

/* The plotted loop repeats its first point so the curve closes. */
static float g_px[ENGINE_TRACE_CAPACITY + 1];
static float g_py[ENGINE_TRACE_CAPACITY + 1];

static const ImVec4 REF_COLOR(0.75f, 0.75f, 0.75f, 0.75f);
static const ImVec4 MARK_COLOR(1.0f, 1.0f, 1.0f, 0.95f);

/* Index of the loop point at cylinder-local angle `deg` (nearest at or after
 * it; the last point if none). */
static int index_at(const PvLoop *l, double deg) {
  for (int k = 0; k < l->n; k++) {
    if ((double)l->angle_deg[k] >= deg) {
      return k;
    }
  }
  return l->n - 1;
}

static void plot_marker(const char *label, ImPlotMarker m, float size, double x,
                        double y, const ImVec4 &fill) {
  ImPlotSpec spec;
  spec.Marker = m;
  spec.MarkerSize = size;
  spec.MarkerFillColor = fill;
  spec.LineColor = fill; /* legend swatch */
  spec.MarkerLineColor = ImVec4(0.0f, 0.0f, 0.0f, 0.8f);
  ImPlot::PlotScatter(label, &x, &y, 1, spec);
}

static void plot_loops(const EngineConfig *cfg, const CylinderConfig *cyl_cfg,
                       const EngineTrace *src, int nc, float height) {
  if (!ImPlot::BeginPlot("Indicator diagram", ImVec2(-1.0f, height),
                         ImPlotFlags_NoMouseText)) {
    return;
  }
  const ImPlotAxisFlags fit =
      g.auto_fit ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None;
  ImPlot::SetupAxes("cylinder volume (cc)", "pressure (kPa)", fit, fit);
  if (g.log_axes) {
    ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
    ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
    ImPlot::SetupAxisLimits(ImAxis_X1, 40.0, 600.0, ImPlotCond_Once);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 30.0, 6000.0, ImPlotCond_Once);
  } else {
    ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 600.0, ImPlotCond_Once);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 5000.0, ImPlotCond_Once);
  }
  ImPlot::SetupLegend(ImPlotLocation_NorthEast);

  int ref_cyl = -1;
  for (int c = 0; c < nc; c++) {
    if (g.sel[c]) {
      ref_cyl = ref_cyl < 0 ? c : ref_cyl;
    }
  }

  /* older cycles first so the newest draws on top */
  for (int back = g.cycles - 1; back >= 0; back--) {
    const float alpha = back == 0 ? 1.0f : 0.55f / (float)(back + 1);
    for (int c = 0; c < nc; c++) {
      if (!g.sel[c]) {
        continue;
      }
      PvStats st;
      const int n = pv_loop_build(src, cfg, cyl_cfg ? &cyl_cfg[c] : NULL, c,
                                  back, &g_loop, &st);
      if (n < 2) {
        continue;
      }
      if (back == 0) {
        g_stats[c] = st;
      }
      for (int k = 0; k < n; k++) {
        g_px[k] = g_loop.v_cc[k];
        g_py[k] = g_loop.p_kpa[k];
      }
      g_px[n] = g_loop.v_cc[0];
      g_py[n] = g_loop.p_kpa[0];

      char label[24];
      if (back == 0) {
        snprintf(label, sizeof label, "Cyl %d", c + 1);
      } else {
        snprintf(label, sizeof label, "##cyl%d_back%d", c + 1, back);
      }
      ImVec4 col = CYL_COLORS[c];
      col.w = alpha;
      ImPlotSpec line;
      line.LineColor = col;
      line.LineWeight = back == 0 ? 1.8f : 1.2f;
      /* the cylinders are identified by the colour-coded checkboxes above */
      line.Flags = ImPlotItemFlags_NoLegend |
                   (back != 0 ? ImPlotItemFlags_NoFit : ImPlotItemFlags_None);
      ImPlot::PlotLine(label, g_px, g_py, n + 1, line);

      if (back == 0 && g.show_markers) {
        const int kp = index_at(&g_loop, st.peak_angle_deg < 0.0
                                             ? st.peak_angle_deg + 720.0
                                             : st.peak_angle_deg);
        const int ki = index_at(&g_loop, cfg->geom.ivc_deg);
        const int ke = index_at(&g_loop, cfg->geom.evo_deg);
        plot_marker("Peak pressure", ImPlotMarker_Diamond, 5.0f,
                    g_loop.v_cc[kp], g_loop.p_kpa[kp], MARK_COLOR);
        plot_marker("IVC", ImPlotMarker_Circle, 4.0f, g_loop.v_cc[ki],
                    g_loop.p_kpa[ki], ImVec4(0.55f, 0.85f, 0.90f, 1.0f));
        plot_marker("EVO", ImPlotMarker_Square, 4.0f, g_loop.v_cc[ke],
                    g_loop.p_kpa[ke], ImVec4(1.0f, 0.75f, 0.20f, 1.0f));
      }
    }
  }

  /* references, drawn from the first selected cylinder's own geometry */
  if (g.show_ref && ref_cyl >= 0 && g_stats[ref_cyl].valid) {
    const PvStats &st = g_stats[ref_cyl];

    /* the no-heat-release curve from the closed-valve start state: what the
     * gas would do if compressed and expanded with no combustion */
    const int kSteps = 96;
    static float rx[kSteps + 1], ry[kSteps + 1];
    for (int k = 0; k <= kSteps; k++) {
      const double v = st.v_min_cc + (st.ivc_v_cc - st.v_min_cc) *
                                         (1.0 - (double)k / kSteps);
      rx[k] = (float)v;
      ry[k] = (float)(st.ivc_p_kpa *
                      pow(st.ivc_v_cc / v, PV_MODEL_POLYTROPIC_N));
    }
    char label[48];
    snprintf(label, sizeof label, "Ideal n=%.2f, no heat release",
             PV_MODEL_POLYTROPIC_N);
    ImPlotSpec ref;
    ref.LineColor = REF_COLOR;
    ref.LineWeight = 1.0f;
    ref.Flags = ImPlotItemFlags_NoFit;
    ImPlot::PlotLine(label, rx, ry, kSteps + 1, ref);

    double atm = 101.325;
    ImPlotSpec atm_spec;
    atm_spec.LineColor = ImVec4(0.75f, 0.75f, 0.75f, 0.35f);
    atm_spec.LineWeight = 1.0f;
    atm_spec.Flags = ImPlotItemFlags_NoFit | ImPlotInfLinesFlags_Horizontal;
    ImPlot::PlotInfLines("1 atm", &atm, 1, atm_spec);
  }

  ImPlot::EndPlot();
}

static void stats_table(const EngineConfig *cfg, int nc) {
  (void)cfg;
  const ImGuiTableFlags flags =
      ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
      ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX;
  if (!ImGui::BeginTable("##pv_stats", 10, flags)) {
    return;
  }
  ImGui::TableSetupColumn("Cyl");
  ImGui::TableSetupColumn("Peak kPa");
  ImGui::TableSetupColumn("ATDC deg");
  ImGui::TableSetupColumn("IMEP net");
  ImGui::TableSetupColumn("IMEP gross");
  ImGui::TableSetupColumn("PMEP");
  ImGui::TableSetupColumn("Work J");
  ImGui::TableSetupColumn("Power kW");
  ImGui::TableSetupColumn("n comp.");
  ImGui::TableSetupColumn("CR");
  ImGui::TableHeadersRow();

  double sum_work = 0.0;
  double sum_power = 0.0;
  int shown = 0;
  for (int c = 0; c < nc; c++) {
    if (!g.sel[c]) {
      continue;
    }
    const PvStats &s = g_stats[c];
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextColored(CYL_COLORS[c], "%d", c + 1);
    if (!s.valid) {
      for (int k = 1; k < 10; k++) {
        ImGui::TableNextColumn();
        ImGui::TextDisabled("--");
      }
      continue;
    }
    ImGui::TableNextColumn();
    ImGui::Text("%.0f", s.peak_kpa);
    ImGui::TableNextColumn();
    ImGui::Text("%+.1f", s.peak_angle_deg);
    ImGui::TableNextColumn();
    ImGui::Text("%.0f", s.imep_net_kpa);
    ImGui::TableNextColumn();
    ImGui::Text("%.0f", s.imep_gross_kpa);
    ImGui::TableNextColumn();
    ImGui::Text("%.0f", s.pmep_kpa);
    ImGui::TableNextColumn();
    ImGui::Text("%.1f", s.work_net_j);
    ImGui::TableNextColumn();
    ImGui::Text("%.2f", s.power_kw);
    ImGui::TableNextColumn();
    if (isnan(s.compression_n)) {
      ImGui::TextDisabled("--");
    } else {
      /* a leaking or weak cylinder compresses with a lower index */
      const bool off = fabs(s.compression_n - PV_MODEL_POLYTROPIC_N) > 0.05;
      ImGui::TextColored(off ? ImVec4(1.0f, 0.75f, 0.20f, 1.0f)
                             : ImGui::GetStyle().Colors[ImGuiCol_Text],
                         "%.2f", s.compression_n);
    }
    ImGui::TableNextColumn();
    ImGui::Text("%.2f", s.comp_ratio);
    sum_work += s.work_net_j;
    sum_power += s.power_kw;
    shown++;
  }
  if (shown > 1) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("Sum");
    for (int k = 1; k < 6; k++) {
      ImGui::TableNextColumn();
    }
    ImGui::TableNextColumn();
    ImGui::Text("%.1f", sum_work);
    ImGui::TableNextColumn();
    ImGui::Text("%.2f", sum_power);
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
  }
  ImGui::EndTable();
}

void pv_diagram_panel_draw(bool *open, const EngineTrace *trace,
                           const EngineConfig *cfg,
                           const CylinderConfig *cyl_cfg) {
  ImGui::SetNextWindowSize(ImVec2(680.0f, 640.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_PV_DIAGRAM, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused && trace) {
    memcpy(&g_snap, trace, sizeof g_snap);
    g.snap_ready = true;
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.0f);
  ImGui::SliderInt("Cycles", &g.cycles, 1, MAX_OVERLAY_CYCLES);
  ImGui::SameLine();
  ImGui::Checkbox("Auto fit", &g.auto_fit);
  ImGui::SameLine();
  ImGui::Checkbox("Log-log", &g.log_axes);
  ImGui::SameLine();
  ImGui::Checkbox("Reference", &g.show_ref);
  ImGui::SameLine();
  ImGui::Checkbox("Markers", &g.show_markers);

  const int nc =
      cfg->num_cylinders < 1
          ? 1
          : (cfg->num_cylinders > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS
                                                       : cfg->num_cylinders);
  ImGui::TextDisabled("Show:");
  for (int c = 0; c < nc; c++) {
    ImGui::SameLine();
    char id[40];
    snprintf(id, sizeof id, "%d##pvsel%d", c + 1, c);
    ImGui::PushStyleColor(ImGuiCol_CheckMark, CYL_COLORS[c]);
    ImGui::Checkbox(id, &g.sel[c]);
    ImGui::PopStyleColor();
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("All")) {
    for (int c = 0; c < nc; c++) {
      g.sel[c] = true;
    }
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("None")) {
    for (int c = 0; c < nc; c++) {
      g.sel[c] = false;
    }
  }

  const EngineTrace *src = (g.paused && g.snap_ready) ? &g_snap : trace;
  if (!src || engine_trace_count(src) < 2) {
    ImGui::TextDisabled("No samples yet -- start the engine (I).");
    ImGui::End();
    return;
  }

  for (int c = 0; c < ENGINE_MAX_CYLINDERS; c++) {
    g_stats[c].valid = 0;
  }

  int nsel = 0;
  for (int c = 0; c < nc; c++) {
    nsel += g.sel[c] ? 1 : 0;
  }
  const float row_h = ImGui::GetTextLineHeightWithSpacing();
  const float table_h = row_h * (float)(nsel + 3) + ImGui::GetStyle().WindowPadding.y;
  float plot_h = ImGui::GetContentRegionAvail().y - table_h;
  if (plot_h < 160.0f) {
    plot_h = 160.0f;
  }

  plot_loops(cfg, cyl_cfg, src, nc, plot_h);

  if (nsel == 0) {
    ImGui::TextDisabled("Select at least one cylinder.");
  } else {
    stats_table(cfg, nc);
  }
  ImGui::TextDisabled("IMEP/PMEP in kPa. Angles are deg after each cylinder's "
                      "own firing TDC.");

  ImGui::End();
}
