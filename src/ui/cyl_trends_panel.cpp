#include "ui/cyl_trends_panel.h"

#include <math.h>
#include <stdio.h>

#include "imgui.h"
#include "implot.h"
#include "telemetry/monitor.h"
#include "ui/cyl_colors.h"
#include "ui/panel_names.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

struct MetricDef {
  const char *title;
  const char *unit;
  const char *column; /* tooltip column heading */
  int precision;
  const ChannelRange *range; /* limits; NULL when there are none */
  double full_lo, full_hi;   /* Y range when not auto-fitting */
};

static const MetricDef METRICS[CYLM_COUNT] = {
    {"CHT", "degC", "CHT", 0, &CHANNEL_RANGE_CHT, 0.0, 260.0},
    {"EGT", "degC", "EGT", 0, &CHANNEL_RANGE_EGT, 0.0, 900.0},
    {"Mixture", "lambda", "Lambda", 2, &CHANNEL_RANGE_LAMBDA, 0.7, 1.7},
    {"Misfire", "%", "Misfire %", 0, NULL, 0.0, 100.0},
};

/* Panel state. There is only ever one Cylinder Trends window. */
struct UiState {
  bool paused = false;
  bool follow = true;
  bool auto_y = true;
  bool show_limits = true;
  bool reset_view = false;

  bool snap_ready = false;
  CylTrends snap;

  bool y_valid[CYLM_COUNT] = {};
  double y_lo[CYLM_COUNT] = {};
  double y_hi[CYLM_COUNT] = {};

  bool hover_now = false;
  bool hover_prev = false;
  int hover_k = 0;
  int hover_k_prev = 0;
};

static UiState g;

/* ---- helpers ----------------------------------------------------------- */

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

static ChannelStatus status_of(int metric, double v) {
  if (metric == CYLM_MISFIRE) {
    return v > 50.0 ? CHANNEL_ALERT : (v > 0.0 ? CHANNEL_WARN : CHANNEL_OK);
  }
  return channel_status_for(v, *METRICS[metric].range);
}

static ImVec4 status_color(ChannelStatus st) {
  if (st == CHANNEL_WARN) {
    return COL_CAUTION;
  }
  if (st == CHANNEL_ALERT) {
    return COL_WARNING;
  }
  return ImGui::GetStyle().Colors[ImGuiCol_Text];
}

/* Min/max over every cylinder's newest `window_s` seconds (0 = everything);
 * false when there is no data. */
static bool data_extent(const CylTrends *t, int metric, int nc, double dt,
                        double window_s, double *lo, double *hi) {
  bool any = false;
  double mn = 0.0;
  double mx = 0.0;
  for (int c = 0; c < nc; c++) {
    const History *h = &t->hist[metric][c];
    const int n = history_count(h);
    if (n == 0) {
      continue;
    }
    int first = 0;
    if (window_s > 0.0) {
      const int span = (int)ceil(window_s / dt) + 1;
      first = n - span > 0 ? n - span : 0;
    }
    for (int i = first; i < n; i++) {
      const double v = history_at(h, i);
      if (!any) {
        mn = mx = v;
        any = true;
      } else {
        mn = v < mn ? v : mn;
        mx = v > mx ? v : mx;
      }
    }
  }
  *lo = mn;
  *hi = mx;
  return any;
}

static void limit_line(const char *id, double y, ImVec4 color) {
  color.w = 0.85f;
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.LineWeight = 1.0f;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit |
               ImPlotInfLinesFlags_Horizontal;
  ImPlot::PlotInfLines(id, &y, 1, spec);
}

static void draw_limits(const ChannelRange &r) {
  if (r.has_warn_lo) {
    limit_line("##warn_lo", r.warn_lo, COL_CAUTION);
  }
  if (r.has_warn_hi) {
    limit_line("##warn_hi", r.warn_hi, COL_CAUTION);
  }
  if (r.has_alert_lo) {
    limit_line("##alert_lo", r.alert_lo, COL_WARNING);
  }
  if (r.has_alert_hi) {
    limit_line("##alert_hi", r.alert_hi, COL_WARNING);
  }
}

/* ---- one metric's plot ------------------------------------------------- */

static void plot_metric(int m, const CylTrends *t, int nc, double dt,
                        double max_span, bool first_row, bool bottom_row) {
  const MetricDef &d = METRICS[m];

  ImPlotFlags flags = ImPlotFlags_NoMouseText;
  if (!first_row) {
    flags |= ImPlotFlags_NoLegend;
  }
  if (!ImPlot::BeginPlot(d.title, ImVec2(-1.0f, -1.0f), flags)) {
    return;
  }

  const bool live_follow = g.follow && !g.paused;
  const ImPlotCond xcond =
      (live_follow || g.reset_view) ? ImPlotCond_Always : ImPlotCond_Once;

  ImPlot::SetupAxes(bottom_row ? "seconds" : NULL, d.unit,
                    bottom_row ? ImPlotAxisFlags_None
                               : ImPlotAxisFlags_NoTickLabels,
                    ImPlotAxisFlags_None);
  if (first_row) {
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
  }
  ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, -max_span - dt, dt);
  ImPlot::SetupAxisLimits(ImAxis_X1, -max_span, 0.0, xcond);

  double lo = 0.0;
  double hi = 0.0;
  if (g.auto_y && data_extent(t, m, nc, dt, 0.0, &lo, &hi)) {
    /* pad the extent, keep a minimum span, ease toward the target */
    const double min_span = 0.05 * (d.full_hi - d.full_lo);
    const double span = (hi - lo) > min_span ? (hi - lo) : min_span;
    const double mid = 0.5 * (hi + lo);
    const double target_lo = mid - span * 0.6;
    const double target_hi = mid + span * 0.6;
    if (!g.y_valid[m] || g.reset_view) {
      g.y_lo[m] = target_lo;
      g.y_hi[m] = target_hi;
      g.y_valid[m] = true;
    } else {
      const double a = 1.0 - exp(-(double)ImGui::GetIO().DeltaTime * 8.0);
      g.y_lo[m] += (target_lo - g.y_lo[m]) * a;
      g.y_hi[m] += (target_hi - g.y_hi[m]) * a;
    }
    ImPlot::SetupAxisLimits(ImAxis_Y1, g.y_lo[m], g.y_hi[m], ImPlotCond_Always);
  } else {
    g.y_valid[m] = false;
    ImPlot::SetupAxisLimits(ImAxis_Y1, d.full_lo, d.full_hi,
                            g.reset_view ? ImPlotCond_Always : ImPlotCond_Once);
  }

  if (g.show_limits && d.range) {
    draw_limits(*d.range);
  }

  int max_n = 0;
  for (int c = 0; c < nc; c++) {
    const History *h = &t->hist[m][c];
    const int n = history_count(h);
    max_n = n > max_n ? n : max_n;
    if (n == 0) {
      continue;
    }
    SeriesRef ref = {h, dt};
    char label[16];
    snprintf(label, sizeof label, "Cyl %d", c + 1);
    ImPlotSpec line;
    line.LineColor = CYL_COLORS[c];
    line.LineWeight = 1.6f;
    ImPlot::PlotLineG(label, series_point, &ref, n, line);
  }

  /* hover: crosshair + a dot on each cylinder at the age chosen last frame */
  if (g.hover_prev && max_n > 0 && g.hover_k_prev < max_n) {
    double x = -(double)g.hover_k_prev * dt;
    ImPlotSpec cross;
    cross.LineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.35f);
    cross.LineWeight = 1.0f;
    cross.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
    ImPlot::PlotInfLines("##hover_x", &x, 1, cross);

    for (int c = 0; c < nc; c++) {
      const History *h = &t->hist[m][c];
      const int n = history_count(h);
      if (n == 0 || g.hover_k_prev >= n) {
        continue;
      }
      double y = history_at(h, n - 1 - g.hover_k_prev);
      char id[16];
      snprintf(id, sizeof id, "##hp%d", c);
      ImPlotSpec dot;
      dot.Marker = ImPlotMarker_Circle;
      dot.MarkerSize = 4.0f;
      dot.MarkerFillColor = CYL_COLORS[c];
      dot.MarkerLineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.9f);
      dot.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
      ImPlot::PlotScatter(id, &x, &y, 1, dot);
    }
  }

  if (ImPlot::IsPlotHovered() && max_n > 0) {
    const ImPlotPoint mp = ImPlot::GetPlotMousePos();
    int age = (int)lround(-mp.x / dt);
    age = age < 0 ? 0 : (age > max_n - 1 ? max_n - 1 : age);
    g.hover_now = true;
    g.hover_k = age;
  }

  ImPlot::EndPlot();
}

/* ---- tooltip + toolbar ------------------------------------------------- */

static void hover_tooltip(const CylTrends *t, int nc, double dt) {
  ImGui::BeginTooltip();
  ImGui::TextDisabled("t = %+.1f s", -(double)g.hover_k * dt);
  if (ImGui::BeginTable("##tt", 1 + CYLM_COUNT, ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("Cyl");
    for (int m = 0; m < CYLM_COUNT; m++) {
      ImGui::TableSetupColumn(METRICS[m].column);
    }
    ImGui::TableHeadersRow();
    for (int c = 0; c < nc; c++) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(CYL_COLORS[c], "%d", c + 1);
      for (int m = 0; m < CYLM_COUNT; m++) {
        const History *h = &t->hist[m][c];
        const int n = history_count(h);
        const int idx = n - 1 - g.hover_k;
        ImGui::TableSetColumnIndex(1 + m);
        if (idx < 0) {
          ImGui::TextDisabled("--");
          continue;
        }
        const double v = history_at(h, idx);
        ImGui::PushStyleColor(ImGuiCol_Text, status_color(status_of(m, v)));
        ImGui::Text("%.*f", METRICS[m].precision, v);
        ImGui::PopStyleColor();
      }
    }
    ImGui::EndTable();
  }
  ImGui::EndTooltip();
}

/* Keeps toolbar items on one line while they fit, otherwise wraps. */
static void wrap_next(float next_width) {
  const ImGuiStyle &style = ImGui::GetStyle();
  const float right =
      ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - style.WindowPadding.x;
  if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + next_width < right) {
    ImGui::SameLine();
  }
}

static float checkbox_width(const char *label) {
  const ImGuiStyle &s = ImGui::GetStyle();
  return ImGui::GetFrameHeight() + s.ItemInnerSpacing.x +
         ImGui::CalcTextSize(label).x;
}

static float button_width(const char *label) {
  return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
}

/* ---- panel ------------------------------------------------------------- */

void cyl_trends_panel_draw(bool *open, const CylTrends *t, int num_cyl,
                           double sample_period_s) {
  ImGui::SetNextWindowSize(ImVec2(640.0f, 700.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_CYL_TRENDS, open)) {
    ImGui::End();
    return;
  }

  const int nc = num_cyl < 1 ? 1
                             : (num_cyl > ENGINE_MAX_CYLINDERS
                                    ? ENGINE_MAX_CYLINDERS
                                    : num_cyl);
  const double dt = sample_period_s;
  const double max_span = (double)CYL_TRENDS_CAP * dt;

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    cyl_trends_snapshot(&g.snap, t);
    g.snap_ready = true;
  }
  wrap_next(checkbox_width("Follow"));
  ImGui::BeginDisabled(g.paused);
  ImGui::Checkbox("Follow", &g.follow);
  ImGui::EndDisabled();
  wrap_next(checkbox_width("Auto Y"));
  ImGui::Checkbox("Auto Y", &g.auto_y);
  wrap_next(checkbox_width("Limits"));
  ImGui::Checkbox("Limits", &g.show_limits);
  wrap_next(button_width("Reset view"));
  if (ImGui::Button("Reset view")) {
    g.follow = true;
    g.auto_y = true;
    g.reset_view = true;
  }

  const CylTrends *src = (g.paused && g.snap_ready) ? &g.snap : t;

  g.hover_now = false;
  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  if (ImPlot::BeginSubplots("##cyl_trends", CYLM_COUNT, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    for (int m = 0; m < CYLM_COUNT; m++) {
      plot_metric(m, src, nc, dt, max_span, m == 0, m == CYLM_COUNT - 1);
    }
    ImPlot::EndSubplots();
  }
  ImPlot::PopStyleVar(2);

  if (g.hover_now) {
    hover_tooltip(src, nc, dt);
  }
  g.hover_prev = g.hover_now;
  g.hover_k_prev = g.hover_k;
  g.reset_view = false;

  ImGui::End();
}
