#include "ui/trends_panel.h"

#include "imgui.h"
#include "implot.h"
#include "telemetry/monitor.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

struct SeriesRef {
  const History *hist;
  double dt;
};

/* x = seconds relative to the newest sample (<= 0), y = the stored value. */
static ImPlotPoint history_point(int idx, void *user) {
  const SeriesRef *ref = (const SeriesRef *)user;
  const int n = history_count(ref->hist);
  return ImPlotPoint((double)(idx - (n - 1)) * ref->dt,
                     history_at(ref->hist, idx));
}

struct TrendDef {
  const char *title;
  const char *unit;
  const History *hist;
  const ChannelRange *range; /* limits, on the channel's own scale */
  double range_scale;        /* range -> plotted units (e.g. 100 for SoC %) */
};

static void limit_line(const char *id, double y, const ImVec4 &color) {
  ImPlotSpec spec;
  spec.LineColor = color;
  spec.Flags = ImPlotItemFlags_NoLegend | ImPlotItemFlags_NoFit |
               ImPlotInfLinesFlags_Horizontal;
  ImPlot::PlotInfLines(id, &y, 1, spec);
}

static void plot_trend(const TrendDef &d, double dt, double span_s) {
  if (!ImPlot::BeginPlot(d.title, ImVec2(-1.0f, -1.0f), ImPlotFlags_NoLegend)) {
    return;
  }
  const ChannelRange &r = *d.range;
  const double k = d.range_scale;

  ImPlot::SetupAxes(NULL, d.unit);
  ImPlot::SetupAxisLimits(ImAxis_X1, -span_s, 0.0, ImPlotCond_Once);
  ImPlot::SetupAxisLimits(ImAxis_Y1, r.lo * k, r.hi * k, ImPlotCond_Once);

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

  SeriesRef ref = {d.hist, dt};
  ImPlotSpec line;
  line.LineWeight = 1.5f;
  ImPlot::PlotLineG(d.title, history_point, &ref, history_count(d.hist), line);

  ImPlot::EndPlot();
}

void trends_panel_draw(bool *open, const Trends *t, double sample_period_s) {
  ImGui::SetNextWindowSize(ImVec2(620.0f, 640.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Trends", open)) {
    ImGui::End();
    return;
  }

  const TrendDef defs[] = {
      {"RPM", "rpm", &t->rpm, &CHANNEL_RANGE_RPM, 1.0},
      {"CHT", "degC", &t->cht, &CHANNEL_RANGE_CHT, 1.0},
      {"EGT", "degC", &t->egt, &CHANNEL_RANGE_EGT, 1.0},
      {"Oil pressure", "kPa", &t->oil_press, &CHANNEL_RANGE_OIL_PRESS, 1.0},
      {"Battery", "%", &t->batt_pct, &CHANNEL_RANGE_BATT_SOC, 100.0},
  };
  const int n = (int)(sizeof defs / sizeof defs[0]);
  const double span_s = (double)TRENDS_CAP * sample_period_s;

  ImGui::TextDisabled("x: seconds before now   amber = caution   red = warning");

  if (ImPlot::BeginSubplots("##trends", n, 1, ImVec2(-1.0f, -1.0f),
                            ImPlotSubplotFlags_LinkAllX)) {
    for (int i = 0; i < n; i++) {
      plot_trend(defs[i], sample_period_s, span_s);
    }
    ImPlot::EndSubplots();
  }

  ImGui::End();
}
