#include "ui/trends_panel.h"

#include <math.h>
#include <stdio.h>

#include "imgui.h"
#include "implot.h"
#include "telemetry/monitor.h"
#include "ui/panel_names.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

enum { CH_RPM, CH_CHT, CH_EGT, CH_OIL, CH_BATT, CH_COUNT };

struct ChannelDef {
  const char *title;
  const char *unit;
  int precision;
  const ChannelRange *range; /* limits, on the channel's own scale */
  double range_scale;        /* channel scale -> plotted units (SoC: 100) */
  ImVec4 color;              /* identity colour (distinct from caution/warning) */
  bool is_rpm;               /* uses channel_rpm_status (stale while cranking) */
};

static const ChannelDef CHANNELS[CH_COUNT] = {
    {"RPM", "rpm", 0, &CHANNEL_RANGE_RPM, 1.0, ImVec4(0.34f, 0.71f, 0.91f, 1.f),
     true},
    {"CHT", "degC", 0, &CHANNEL_RANGE_CHT, 1.0,
     ImVec4(0.00f, 0.72f, 0.55f, 1.f), false},
    {"EGT", "degC", 0, &CHANNEL_RANGE_EGT, 1.0,
     ImVec4(0.80f, 0.47f, 0.65f, 1.f), false},
    {"Oil pressure", "kPa", 0, &CHANNEL_RANGE_OIL_PRESS, 1.0,
     ImVec4(0.40f, 0.60f, 1.00f, 1.f), false},
    {"Battery", "%", 0, &CHANNEL_RANGE_BATT_SOC, 100.0,
     ImVec4(0.80f, 0.80f, 0.80f, 1.f), false},
};

/* Panel state. There is only ever one Trends window. */
struct UiState {
  bool paused = false;
  bool follow = true;      /* X window slides with "now" */
  bool auto_y = true;      /* Y fits the visible data */
  bool show_limits = true; /* caution / warning bands + lines */
  int window_idx = 2;      /* index into WINDOW_FRACTIONS */
  bool reset_view = false; /* re-apply axis limits this frame */

  /* frozen copy of the series while paused */
  bool snap_ready = false;
  double snap_buf[CH_COUNT][TRENDS_CAP];
  History snap[CH_COUNT];

  /* smoothed auto-Y limits per channel */
  bool y_valid[CH_COUNT] = {};
  double y_lo[CH_COUNT] = {};
  double y_hi[CH_COUNT] = {};

  /* hover: age (in samples before newest) under the mouse */
  bool hover_now = false;
  bool hover_prev = false; /* last frame's, so every plot can draw the line */
  int hover_k = 0;
  int hover_k_prev = 0;

  double copied_until = 0.0; /* ImGui time until which "copied" is shown */
  int copied_rows = 0;
};

static UiState g;

static const double WINDOW_FRACTIONS[] = {1.0 / 3.0, 2.0 / 3.0, 1.0};
static const int WINDOW_COUNT = (int)(sizeof WINDOW_FRACTIONS /
                                      sizeof WINDOW_FRACTIONS[0]);

/* ---- helpers ----------------------------------------------------------- */

struct SeriesRef {
  const History *hist;
  double dt;
  double y_const; /* used by baseline_point */
};

/* x = seconds relative to the newest sample (<= 0), y = the stored value. */
static ImPlotPoint series_point(int idx, void *user) {
  const SeriesRef *ref = (const SeriesRef *)user;
  const int n = history_count(ref->hist);
  return ImPlotPoint((double)(idx - (n - 1)) * ref->dt,
                     history_at(ref->hist, idx));
}

static ImPlotPoint baseline_point(int idx, void *user) {
  const SeriesRef *ref = (const SeriesRef *)user;
  const int n = history_count(ref->hist);
  return ImPlotPoint((double)(idx - (n - 1)) * ref->dt, ref->y_const);
}

static ChannelStatus status_of(const ChannelDef &c, double plotted) {
  const double raw = plotted / c.range_scale;
  return c.is_rpm ? channel_rpm_status(raw) : channel_status_for(raw, *c.range);
}

/* Identity colour, replaced by caution / warning colour when out of limits. */
static ImVec4 tinted(const ChannelDef &c, ChannelStatus st) {
  if (st == CHANNEL_WARN) {
    return COL_CAUTION;
  }
  if (st == CHANNEL_ALERT) {
    return COL_WARNING;
  }
  return c.color;
}

static void snapshot_from(const History *const live[CH_COUNT]) {
  for (int c = 0; c < CH_COUNT; c++) {
    history_init(&g.snap[c], g.snap_buf[c], TRENDS_CAP);
    const int n = history_count(live[c]);
    for (int i = 0; i < n; i++) {
      history_push(&g.snap[c], history_at(live[c], i));
    }
  }
  g.snap_ready = true;
}

/* Min/max of the newest `window_s` seconds; false when there is no data. */
static bool data_extent(const History *h, double dt, double window_s,
                        double *lo, double *hi) {
  const int n = history_count(h);
  if (n == 0) {
    return false;
  }
  int first = 0;
  if (window_s > 0.0) {
    const int span = (int)ceil(window_s / dt) + 1;
    first = n - span > 0 ? n - span : 0;
  }
  double mn = history_at(h, first);
  double mx = mn;
  for (int i = first + 1; i < n; i++) {
    const double v = history_at(h, i);
    mn = v < mn ? v : mn;
    mx = v > mx ? v : mx;
  }
  *lo = mn;
  *hi = mx;
  return true;
}

static void band(const ImPlotRect &lim, double y0, double y1, ImVec4 color,
                 float alpha) {
  if (y1 <= lim.Y.Min || y0 >= lim.Y.Max || y1 <= y0) {
    return;
  }
  y0 = y0 < lim.Y.Min ? lim.Y.Min : y0;
  y1 = y1 > lim.Y.Max ? lim.Y.Max : y1;
  color.w = alpha;
  ImDrawList *dl = ImPlot::GetPlotDrawList();
  const ImVec2 a = ImPlot::PlotToPixels(lim.X.Min, y1);
  const ImVec2 b = ImPlot::PlotToPixels(lim.X.Max, y0);
  dl->AddRectFilled(a, b, ImGui::ColorConvertFloat4ToU32(color));
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

static void draw_limits(const ChannelDef &d, const ImPlotRect &lim) {
  const ChannelRange &r = *d.range;
  const double k = d.range_scale;
  const float band_alpha = 0.13f;

  ImPlot::PushPlotClipRect();
  /* high side: caution up to warning, warning above */
  if (r.has_warn_hi) {
    band(lim, r.warn_hi * k, r.has_alert_hi ? r.alert_hi * k : lim.Y.Max,
         COL_CAUTION, band_alpha);
  }
  if (r.has_alert_hi) {
    band(lim, r.alert_hi * k, lim.Y.Max, COL_WARNING, band_alpha);
  }
  /* low side: warning below, caution up to the caution limit. RPM at/below
   * cranking speed is "no data", not a warning, so that band is skipped. */
  if (r.has_warn_lo) {
    band(lim, r.has_alert_lo ? r.alert_lo * k : lim.Y.Min, r.warn_lo * k,
         COL_CAUTION, band_alpha);
  }
  if (r.has_alert_lo && !d.is_rpm) {
    band(lim, lim.Y.Min, r.alert_lo * k, COL_WARNING, band_alpha);
  }
  ImPlot::PopPlotClipRect();

  if (r.has_warn_lo) {
    limit_line("##warn_lo", r.warn_lo * k, COL_CAUTION);
  }
  if (r.has_warn_hi) {
    limit_line("##warn_hi", r.warn_hi * k, COL_CAUTION);
  }
  if (r.has_alert_lo) {
    limit_line("##alert_lo", r.alert_lo * k, COL_WARNING);
  }
  if (r.has_alert_hi) {
    limit_line("##alert_hi", r.alert_hi * k, COL_WARNING);
  }
}

/* ---- one channel's plot ------------------------------------------------ */

static void plot_channel(int c, const History *h, double dt, double window_s,
                         double max_span, bool bottom_row) {
  const ChannelDef &d = CHANNELS[c];
  const ChannelRange &r = *d.range;
  const double k = d.range_scale;

  if (!ImPlot::BeginPlot(d.title, ImVec2(-1.0f, -1.0f),
                         ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
    return;
  }

  const bool live_follow = g.follow && !g.paused;
  const ImPlotCond xcond =
      (live_follow || g.reset_view) ? ImPlotCond_Always : ImPlotCond_Once;

  ImPlot::SetupAxes(bottom_row ? "seconds" : NULL, d.unit,
                    bottom_row ? ImPlotAxisFlags_None
                               : ImPlotAxisFlags_NoTickLabels,
                    ImPlotAxisFlags_None);
  ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, -max_span - dt, dt);
  ImPlot::SetupAxisLimits(ImAxis_X1, -window_s, 0.0, xcond);

  double lo = 0.0;
  double hi = 0.0;
  if (g.auto_y &&
      data_extent(h, dt, live_follow ? window_s : 0.0, &lo, &hi)) {
    /* pad the data extent, keep a minimum span so a flat line isn't blown up
     * to noise, and ease toward the target so the axis doesn't jitter */
    const double min_span = 0.05 * (r.hi - r.lo) * k;
    const double span = (hi - lo) > min_span ? (hi - lo) : min_span;
    const double mid = 0.5 * (hi + lo);
    const double target_lo = mid - span * 0.6;
    const double target_hi = mid + span * 0.6;
    if (!g.y_valid[c] || g.reset_view) {
      g.y_lo[c] = target_lo;
      g.y_hi[c] = target_hi;
      g.y_valid[c] = true;
    } else {
      const double a = 1.0 - exp(-(double)ImGui::GetIO().DeltaTime * 8.0);
      g.y_lo[c] += (target_lo - g.y_lo[c]) * a;
      g.y_hi[c] += (target_hi - g.y_hi[c]) * a;
    }
    ImPlot::SetupAxisLimits(ImAxis_Y1, g.y_lo[c], g.y_hi[c], ImPlotCond_Always);
  } else {
    g.y_valid[c] = false;
    ImPlot::SetupAxisLimits(ImAxis_Y1, r.lo * k, r.hi * k,
                            g.reset_view ? ImPlotCond_Always : ImPlotCond_Once);
  }

  const ImPlotRect lim = ImPlot::GetPlotLimits();
  const int n = history_count(h);

  if (g.show_limits) {
    draw_limits(d, lim);
  }

  if (n > 0) {
    SeriesRef ref = {h, dt, lim.Y.Min};
    ImPlotSpec fill;
    fill.FillColor = d.color;
    fill.FillAlpha = 0.16f;
    fill.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
    ImPlot::PlotShadedG("##fill", series_point, &ref, baseline_point, &ref, n,
                        fill);

    ImPlotSpec line;
    line.LineColor = d.color;
    line.LineWeight = 1.8f;
    ImPlot::PlotLineG(d.title, series_point, &ref, n, line);

    /* latest-value tag at the right edge, tinted by status */
    const double last = history_at(h, n - 1);
    ImPlot::Annotation(0.0, last, tinted(d, status_of(d, last)),
                       ImVec2(-10.0f, 0.0f), true, "%.*f", d.precision, last);
  }

  /* hover: crosshair + point at the age chosen (last frame) by any plot */
  if (g.hover_prev && n > 0 && g.hover_k_prev < n) {
    double x = -(double)g.hover_k_prev * dt;
    double y = history_at(h, n - 1 - g.hover_k_prev);
    ImPlotSpec cross;
    cross.LineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.35f);
    cross.LineWeight = 1.0f;
    cross.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
    ImPlot::PlotInfLines("##hover_x", &x, 1, cross);

    ImPlotSpec dot;
    dot.Marker = ImPlotMarker_Circle;
    dot.MarkerSize = 4.5f;
    dot.MarkerFillColor = d.color;
    dot.MarkerLineColor = ImVec4(1.0f, 1.0f, 1.0f, 0.9f);
    dot.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit;
    ImPlot::PlotScatter("##hover_pt", &x, &y, 1, dot);
  }

  if (ImPlot::IsPlotHovered() && n > 0) {
    const ImPlotPoint mp = ImPlot::GetPlotMousePos();
    int age = (int)lround(-mp.x / dt);
    age = age < 0 ? 0 : (age > n - 1 ? n - 1 : age);
    g.hover_now = true;
    g.hover_k = age;
  }

  ImPlot::EndPlot();
}

/* ---- tooltip + toolbar ------------------------------------------------- */

static void hover_tooltip(const History *const src[CH_COUNT], double dt) {
  ImGui::BeginTooltip();
  ImGui::TextDisabled("t = %+.1f s", -(double)g.hover_k * dt);
  if (ImGui::BeginTable("##tt", 3, ImGuiTableFlags_SizingFixedFit)) {
    for (int c = 0; c < CH_COUNT; c++) {
      const int n = history_count(src[c]);
      const int idx = n - 1 - g.hover_k;
      if (idx < 0) {
        continue;
      }
      const ChannelDef &d = CHANNELS[c];
      const double v = history_at(src[c], idx);
      const ChannelStatus st = status_of(d, v);

      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(d.color, "%s", d.title);
      ImGui::TableSetColumnIndex(1);
      ImGui::PushStyleColor(ImGuiCol_Text, st == CHANNEL_OK ? ImGui::GetStyle().Colors[ImGuiCol_Text]
                                                             : tinted(d, st));
      ImGui::Text("%.*f", d.precision, v);
      ImGui::PopStyleColor();
      ImGui::TableSetColumnIndex(2);
      ImGui::TextDisabled("%s", d.unit);
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

static void copy_csv(const History *const src[CH_COUNT], double dt) {
  const int n = history_count(src[0]);
  ImGuiTextBuffer buf;
  buf.appendf("t_s,rpm,cht_c,egt_c,oil_press_kpa,batt_pct\n");
  for (int i = 0; i < n; i++) {
    buf.appendf("%.2f", (double)(i - (n - 1)) * dt);
    for (int c = 0; c < CH_COUNT; c++) {
      buf.appendf(",%.3f", history_at(src[c], i));
    }
    buf.appendf("\n");
  }
  ImGui::SetClipboardText(buf.c_str());
  g.copied_rows = n;
  g.copied_until = ImGui::GetTime() + 2.0;
}

/* ---- panel ------------------------------------------------------------- */

void trends_panel_draw(bool *open, const Trends *t, double sample_period_s) {
  ImGui::SetNextWindowSize(ImVec2(640.0f, 660.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_TRENDS, open)) {
    ImGui::End();
    return;
  }

  const History *const live[CH_COUNT] = {&t->rpm, &t->cht, &t->egt,
                                         &t->oil_press, &t->batt_pct};
  const double dt = sample_period_s;
  const double max_span = (double)TRENDS_CAP * dt;

  /* toolbar */
  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    snapshot_from(live);
  }
  wrap_next(checkbox_width("Follow"));
  ImGui::BeginDisabled(g.paused);
  ImGui::Checkbox("Follow", &g.follow);
  ImGui::EndDisabled();
  wrap_next(checkbox_width("Auto Y"));
  ImGui::Checkbox("Auto Y", &g.auto_y);
  wrap_next(checkbox_width("Limits"));
  ImGui::Checkbox("Limits", &g.show_limits);

  char cur[16];
  snprintf(cur, sizeof cur, "%.0f s",
           WINDOW_FRACTIONS[g.window_idx] * max_span);
  wrap_next(ImGui::GetFontSize() * 5.0f);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
  if (ImGui::BeginCombo("##window", cur)) {
    for (int i = 0; i < WINDOW_COUNT; i++) {
      char label[16];
      snprintf(label, sizeof label, "%.0f s", WINDOW_FRACTIONS[i] * max_span);
      if (ImGui::Selectable(label, i == g.window_idx)) {
        g.window_idx = i;
        g.reset_view = true;
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("visible time window");
  }

  wrap_next(button_width("Reset view"));
  if (ImGui::Button("Reset view")) {
    g.follow = true;
    g.auto_y = true;
    g.reset_view = true;
  }

  const History *src[CH_COUNT];
  for (int c = 0; c < CH_COUNT; c++) {
    src[c] = (g.paused && g.snap_ready) ? &g.snap[c] : live[c];
  }

  wrap_next(button_width("Copy CSV"));
  if (ImGui::Button("Copy CSV")) {
    copy_csv(src, dt);
  }
  if (ImGui::GetTime() < g.copied_until) {
    ImGui::SameLine();
    ImGui::TextDisabled("copied %d rows", g.copied_rows);
  }

  ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(6.0f, 4.0f));
  ImPlot::PushStyleVar(ImPlotStyleVar_MinorAlpha, 0.12f);
  ImPlot::PushStyleColor(ImPlotCol_PlotBg, ImVec4(0.06f, 0.07f, 0.09f, 1.0f));

  g.hover_now = false;
  const double window_s = WINDOW_FRACTIONS[g.window_idx] * max_span;
  if (ImPlot::BeginSubplots("##trends", CH_COUNT, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    for (int c = 0; c < CH_COUNT; c++) {
      plot_channel(c, src[c], dt, window_s, max_span, c == CH_COUNT - 1);
    }
    ImPlot::EndSubplots();
  }

  ImPlot::PopStyleColor();
  ImPlot::PopStyleVar(2);

  if (g.hover_now) {
    hover_tooltip(src, dt);
  }
  g.hover_prev = g.hover_now;
  g.hover_k_prev = g.hover_k;
  g.reset_view = false;

  ImGui::End();
}
