#include "ui/profiler_panel.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <SDL3/SDL.h>

#include "imgui.h"
#include "implot.h"
#include "ui/panel_names.h"

#define DEFAULT_TARGET_MS (1000.0 / 60.0)
#define CLAMPED_FRAME_MS 50.0

static const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
static const ImVec4 kWarn(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 kBad(1.00f, 0.35f, 0.30f, 1.0f);
static const ImVec4 kFrame(0.92f, 0.92f, 0.92f, 1.0f);
static const ImVec4 kPhysics(0.35f, 0.65f, 0.95f, 1.0f);
static const ImVec4 kUi(0.85f, 0.45f, 0.75f, 1.0f);
static const ImVec4 kRender(0.55f, 0.85f, 0.90f, 1.0f);

static const int kWindows[] = {60, 120, 300, 600};
static const char *kWindowLabels[] = {"60 frames", "120 frames", "300 frames",
                                      "600 frames"};

struct UiState {
  bool paused = false;
  bool snap_ready = false;
  int window_idx = 1;
  char status[160] = "";
};

static UiState g;
static Profiler g_snap; /* frozen copy while paused (~120 KB: keep it static) */

/* Plot series, rebuilt each frame. */
static float g_x[PROF_HISTORY];
static float g_frame[PROF_HISTORY];
static float g_phys[PROF_HISTORY];
static float g_ui[PROF_HISTORY];
static float g_render[PROF_HISTORY];

static ImVec4 load_color(double frac) {
  return frac < 0.6 ? kGood : (frac < 0.85 ? kWarn : kBad);
}

static void kv(const char *key, const ImVec4 *col, const char *fmt, ...)
    IM_FMTARGS(3);
static void kv(const char *key, const ImVec4 *col, const char *fmt, ...) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextDisabled("%s", key);
  ImGui::TableNextColumn();
  va_list args;
  va_start(args, fmt);
  if (col) {
    ImGui::PushStyleColor(ImGuiCol_Text, *col);
  }
  ImGui::TextWrappedV(fmt, args);
  if (col) {
    ImGui::PopStyleColor();
  }
  va_end(args);
}

static bool kv_begin(const char *id) {
  if (!ImGui::BeginTable(id, 2,
                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                             ImGuiTableFlags_SizingStretchProp)) {
    return false;
  }
  ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 13.0f);
  ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch);
  return true;
}

static void section(const char *title) {
  ImGui::Spacing();
  ImGui::TextColored(ImVec4(0.35f, 0.72f, 1.0f, 1.0f), "%s", title);
  ImGui::Separator();
}

static void export_csv(const Profiler *p) {
  SDL_CreateDirectory("runs");
  char path[96];
  snprintf(path, sizeof path, "runs/profile_%llu.csv",
           (unsigned long long)SDL_GetTicks());
  FILE *f = fopen(path, "w");
  if (!f) {
    snprintf(g.status, sizeof g.status, "could not write %s", path);
    return;
  }
  const int n = profiler_write_csv(p, f);
  fclose(f);
  snprintf(g.status, sizeof g.status, "saved %d frames to %s", n, path);
}

static void vline_h(const char *id, double y, ImVec4 col) {
  ImPlotSpec spec;
  spec.LineColor = col;
  spec.LineWeight = 1.0f;
  spec.Flags = ImPlotItemFlags_NoFit | ImPlotInfLinesFlags_Horizontal;
  ImPlot::PlotInfLines(id, &y, 1, spec);
}

static void line(const char *label, const float *y, int n, ImVec4 col,
                 float weight) {
  ImPlotSpec spec;
  spec.LineColor = col;
  spec.LineWeight = weight;
  ImPlot::PlotLine(label, g_x, y, n, spec);
}

static void plot_history(const Profiler *p, const ProfilerPanelInfo &info,
                         int n, double target_ms, float height) {
  const int first = p->count - n;
  for (int i = 0; i < n; i++) {
    const ProfFrame *f = profiler_frame_at(p, first + i);
    g_x[i] = (float)(i - n);
    g_frame[i] = f->frame_ms;
    g_phys[i] =
        f->stage_ms[info.physics_stage] +
        (info.shadow_stage >= 0 ? f->stage_ms[info.shadow_stage] : 0.0f);
    g_ui[i] = f->stage_ms[info.ui_stage];
    g_render[i] =
        f->stage_ms[info.render_stage] + f->stage_ms[info.present_stage];
  }

  if (!ImPlot::BeginPlot("Frame time", ImVec2(-1.0f, height),
                         ImPlotFlags_NoMouseText)) {
    return;
  }
  ImPlot::SetupAxes("frames ago", "ms", ImPlotAxisFlags_None,
                    ImPlotAxisFlags_AutoFit);
  ImPlot::SetupAxisLimits(ImAxis_X1, -(double)n, 0.0, ImPlotCond_Always);
  ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0.0, 1.0e6);
  ImPlot::SetupLegend(ImPlotLocation_North,
                      ImPlotLegendFlags_Horizontal | ImPlotLegendFlags_Outside);

  vline_h("##budget", target_ms, ImVec4(0.35f, 0.85f, 0.45f, 0.55f));
  vline_h("##budget2", target_ms * 2.0, ImVec4(1.0f, 0.35f, 0.30f, 0.45f));
  line("Frame", g_frame, n, kFrame, 1.8f);
  line("Physics", g_phys, n, kPhysics, 1.3f);
  line("UI", g_ui, n, kUi, 1.3f);
  line("Render + present", g_render, n, kRender, 1.3f);
  ImPlot::EndPlot();
}

static void bar_cell(double frac, ImVec4 col, const char *overlay) {
  ImGui::PushStyleColor(ImGuiCol_PlotHistogram, col);
  ImGui::ProgressBar((float)(frac < 0.0 ? 0.0 : (frac > 1.0 ? 1.0 : frac)),
                     ImVec2(-1.0f, ImGui::GetTextLineHeight()), overlay);
  ImGui::PopStyleColor();
}

static void stage_table(const Profiler *p, const ProfilerPanelInfo &info, int n,
                        double frame_avg_ms) {
  if (!ImGui::BeginTable("##prof_stages", 4,
                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                             ImGuiTableFlags_SizingStretchProp)) {
    return;
  }
  ImGui::TableSetupColumn("Stage", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 11.0f);
  ImGui::TableSetupColumn("avg ms", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 5.0f);
  ImGui::TableSetupColumn("worst ms", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 5.5f);
  ImGui::TableSetupColumn("share of frame");
  ImGui::TableHeadersRow();

  double accounted = 0.0;
  for (int s = 0; s < info.first_panel_stage; s++) {
    const double avg = profiler_stage_avg_ms(p, s, n);
    const double worst = profiler_stage_max_ms(p, s, n);
    accounted += avg;
    const double share = frame_avg_ms > 0.0 ? avg / frame_avg_ms : 0.0;
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(p->stage_name[s]);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", avg);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", worst);
    ImGui::TableNextColumn();
    char label[16];
    snprintf(label, sizeof label, "%.1f%%", share * 100.0);
    /* the present stage is mostly waiting for vsync, not work */
    bar_cell(share,
             s == info.present_stage ? ImVec4(0.45f, 0.50f, 0.55f, 1.0f)
                                     : ImVec4(0.35f, 0.65f, 0.95f, 1.0f),
             label);
  }
  /* what no stage claims: event handling, the frame timer, scheduling */
  const double other = frame_avg_ms - accounted;
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextDisabled("Other / events");
  ImGui::TableNextColumn();
  ImGui::TextDisabled("%.3f", other > 0.0 ? other : 0.0);
  ImGui::TableNextColumn();
  ImGui::TextDisabled("--");
  ImGui::TableNextColumn();
  {
    char label[16];
    const double share = frame_avg_ms > 0.0 ? other / frame_avg_ms : 0.0;
    snprintf(label, sizeof label, "%.1f%%", share * 100.0);
    bar_cell(share, ImVec4(0.45f, 0.50f, 0.55f, 1.0f), label);
  }
  ImGui::EndTable();
}

struct PanelRow {
  int stage;
  double avg;
  double worst;
};

static int cmp_row_desc(const void *a, const void *b) {
  const double x = ((const PanelRow *)a)->avg;
  const double y = ((const PanelRow *)b)->avg;
  return (x < y) - (x > y);
}

static void panel_table(const Profiler *p, const ProfilerPanelInfo &info, int n,
                        double ui_avg_ms) {
  PanelRow rows[PROF_MAX_STAGES];
  int nr = 0;
  for (int s = info.first_panel_stage; s < p->stage_count; s++) {
    const double avg = profiler_stage_avg_ms(p, s, n);
    if (avg <= 0.0) {
      continue; /* closed panels cost nothing */
    }
    rows[nr].stage = s;
    rows[nr].avg = avg;
    rows[nr].worst = profiler_stage_max_ms(p, s, n);
    nr++;
  }
  if (nr == 0) {
    ImGui::TextDisabled("-- no panel timings yet --");
    return;
  }
  qsort(rows, (size_t)nr, sizeof rows[0], cmp_row_desc);

  if (!ImGui::BeginTable("##prof_panels", 4,
                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                             ImGuiTableFlags_SizingStretchProp)) {
    return;
  }
  ImGui::TableSetupColumn("Panel", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 11.0f);
  ImGui::TableSetupColumn("avg ms", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 5.0f);
  ImGui::TableSetupColumn("worst ms", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 5.5f);
  ImGui::TableSetupColumn("share of UI");
  ImGui::TableHeadersRow();
  for (int i = 0; i < nr; i++) {
    const double share = ui_avg_ms > 0.0 ? rows[i].avg / ui_avg_ms : 0.0;
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(p->stage_name[rows[i].stage]);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", rows[i].avg);
    ImGui::TableNextColumn();
    ImGui::Text("%.3f", rows[i].worst);
    ImGui::TableNextColumn();
    char label[16];
    snprintf(label, sizeof label, "%.1f%%", share * 100.0);
    bar_cell(share, ImVec4(0.85f, 0.45f, 0.75f, 1.0f), label);
  }
  ImGui::EndTable();
}

void profiler_panel_draw(bool *open, Profiler *live,
                         const ProfilerPanelInfo &info) {
  ImGui::SetNextWindowSize(ImVec2(720.0f, 820.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_PROFILER, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::Checkbox("Pause", &g.paused) && g.paused) {
    memcpy(&g_snap, live, sizeof g_snap);
    g.snap_ready = true;
  }
  const Profiler *p = (g.paused && g.snap_ready) ? &g_snap : live;

  ImGui::SameLine();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0f);
  ImGui::Combo("Average over", &g.window_idx, kWindowLabels,
               (int)(sizeof kWindows / sizeof kWindows[0]));
  ImGui::SameLine();
  ImGui::BeginDisabled(p->count == 0);
  if (ImGui::Button("Export CSV")) {
    export_csv(p);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    profiler_reset(live);
    g.snap_ready = false;
    g.paused = false;
    g.status[0] = 0;
    p = live;
  }
  if (g.status[0]) {
    ImGui::TextDisabled("%s", g.status);
  }

  if (p->count < 2) {
    ImGui::TextDisabled("Collecting frames...");
    ImGui::End();
    return;
  }

  const int n =
      p->count < kWindows[g.window_idx] ? p->count : kWindows[g.window_idx];
  const ProfSummary s = profiler_frame_summary(p, n);
  const double target_ms =
      info.target_frame_ms > 0.0 ? info.target_frame_ms : DEFAULT_TARGET_MS;

  const double present_avg = profiler_stage_avg_ms(p, info.present_stage, n);
  const double phys_avg = profiler_stage_avg_ms(p, info.physics_stage, n);
  const double shadow_avg = info.shadow_stage >= 0
                                ? profiler_stage_avg_ms(p, info.shadow_stage, n)
                                : 0.0;
  const double ui_avg = profiler_stage_avg_ms(p, info.ui_stage, n);

  /* Work the CPU did, leaving out the wait inside present for vsync. */
  const double busy_ms = s.avg - present_avg;
  const double busy_frac = busy_ms / target_ms;

  int hitches = 0;
  int clamped = 0;
  {
    const double hitch_ms =
        s.p50 * 1.5 > s.p50 + 4.0 ? s.p50 * 1.5 : s.p50 + 4.0;
    const int first = p->count - n;
    for (int i = 0; i < n; i++) {
      const double ms = profiler_frame_at(p, first + i)->frame_ms;
      hitches += ms > hitch_ms ? 1 : 0;
      clamped += ms > CLAMPED_FRAME_MS ? 1 : 0;
    }
  }

  section("SUMMARY");
  if (kv_begin("##prof_sum")) {
    const ImVec4 fc = s.p99 <= target_ms * 1.25  ? kGood
                      : s.p99 <= target_ms * 2.0 ? kWarn
                                                 : kBad;
    kv("Frame rate", &fc, "%.1f FPS   (target %.1f Hz, budget %.2f ms)",
       1000.0 / s.avg, 1000.0 / target_ms, target_ms);
    kv("Frame time", NULL,
       "avg %.2f   p50 %.2f   p95 %.2f   p99 %.2f   worst %.2f ms", s.avg,
       s.p50, s.p95, s.p99, s.max);
    const ImVec4 hc = hitches == 0 ? kGood : (hitches < n / 20 ? kWarn : kBad);
    kv("Hitches", &hc, "%d of %d frames noticeably slower than typical",
       hitches, n);
    const ImVec4 bc = load_color(busy_frac);
    kv("CPU headroom", &bc,
       "%.0f%% of the frame budget in use   (%.2f ms of "
       "work, not counting the vsync wait)",
       busy_frac * 100.0, busy_ms);
    if (clamped > 0) {
      kv("Clamped frames", &kBad,
         "%d frame(s) over %.0f ms -- the loop caps dt there, so simulated "
         "time was lost",
         clamped, CLAMPED_FRAME_MS);
    }
    ImGui::EndTable();
  }

  plot_history(p, info, n, target_ms, ImGui::GetTextLineHeight() * 13.0f);

  section("WHERE THE TIME GOES");
  stage_table(p, info, n, s.avg);

  section("PHYSICS LOAD");
  if (kv_begin("##prof_phys")) {
    const double steps = profiler_counter_avg(p, info.steps_counter, n);
    const double subs = profiler_counter_avg(p, info.substeps_counter, n);
    const double shadow_steps =
        profiler_counter_avg(p, info.shadow_steps_counter, n);
    const double sim_s = profiler_counter_avg(p, info.sim_s_counter, n);

    kv("Model steps / frame", NULL, "%.2f   (max %.0f)", steps,
       profiler_counter_max(p, info.steps_counter, n));
    kv("Crank sub-steps / frame", NULL, "%.0f   (max %.0f)", subs,
       profiler_counter_max(p, info.substeps_counter, n));
    if (steps > 0.0) {
      kv("Cost per model step", NULL, "%.3f ms", phys_avg / steps);
    }
    if (subs > 0.0) {
      kv("Cost per crank sub-step", NULL,
         "%.2f us   (RK4 of the crank + "
         "every cylinder, plus trace)",
         phys_avg / subs * 1000.0);
    }
    kv("Shadow model", NULL, "%s%s",
       info.shadow_active ? "running" : "off (no ECU fitted)",
       info.shadow_active ? "  -- a second full engine step per frame" : "");
    if (info.shadow_active && shadow_steps > 0.0) {
      kv("Shadow cost", NULL, "%.3f ms / frame   (%.0f%% of physics)",
         shadow_avg,
         phys_avg > 0.0 ? shadow_avg / (phys_avg + shadow_avg) * 100.0 : 0.0);
    }

    /* Real-time factor: simulated seconds gained per wall second. */
    const double rtf = s.avg > 0.0 ? sim_s / (s.avg / 1000.0) : 0.0;
    if (info.sim_paused) {
      kv("Real-time factor", NULL, "simulation paused");
    } else if (info.sim_speed > 0.0) {
      const double ratio = rtf / info.sim_speed;
      const ImVec4 rc = ratio >= 0.97 ? kGood : (ratio >= 0.85 ? kWarn : kBad);
      kv("Real-time factor", &rc, "%.2fx achieved of %.2fx requested  (%.0f%%)",
         rtf, info.sim_speed, ratio * 100.0);
    }

    /* How fast could the physics alone run if it had the whole frame? */
    const double phys_total = phys_avg + shadow_avg;
    if (sim_s > 0.0 && phys_total > 0.0) {
      const double ms_per_sim_s = phys_total / sim_s;
      kv("Physics cost", NULL, "%.3f ms per simulated second", ms_per_sim_s);
      kv("Physics-only ceiling", NULL,
         "~%.0fx real time (if physics had the whole budget)",
         1000.0 / ms_per_sim_s);
    }
    ImGui::EndTable();
  }

  section("UI COST PER PANEL");
  ImGui::TextDisabled("UI build total: %.2f ms avg, %.1f%% of the frame",
                      ui_avg, s.avg > 0.0 ? ui_avg / s.avg * 100.0 : 0.0);
  panel_table(p, info, n, ui_avg);

  ImGui::Spacing();
  ImGui::TextDisabled("Frames are timed wall-clock from one frame start to the "
                      "next. Present includes the wait for vsync.");

  ImGui::End();
}
