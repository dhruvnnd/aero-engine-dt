#include "ui/readout_panels.h"

#include <float.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"
#include "telemetry/monitor.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

/* Nominal readings keep the normal text colour so colour only ever means
 * caution / warning / no data. */
static ImVec4 status_color(ChannelStatus st) {
  switch (st) {
  case CHANNEL_WARN:
    return COL_CAUTION;
  case CHANNEL_ALERT:
    return COL_WARNING;
  case CHANNEL_STALE:
    return ImGui::GetStyle().Colors[ImGuiCol_TextDisabled];
  case CHANNEL_OK:
  default:
    return ImGui::GetStyle().Colors[ImGuiCol_Text];
  }
}

static const char *status_token(ChannelStatus st) {
  switch (st) {
  case CHANNEL_WARN:
    return "[WARN]";
  case CHANNEL_ALERT:
    return "[ALRM]";
  case CHANNEL_STALE:
    return "[----]";
  case CHANNEL_OK:
  default:
    return "[ OK ]";
  }
}

static ChannelStatus misfire_status(double rate) {
  return rate > 0.5 ? CHANNEL_ALERT : (rate > 0.0 ? CHANNEL_WARN : CHANNEL_OK);
}

static float fraction_of(double v, const ChannelRange &r) {
  double f = (v - r.lo) / (r.hi - r.lo);
  return (float)(f < 0.0 ? 0.0 : (f > 1.0 ? 1.0 : f));
}

static void right_aligned_text(const char *text) {
  const float w = ImGui::CalcTextSize(text).x;
  const float avail = ImGui::GetContentRegionAvail().x;
  if (avail > w) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + avail - w);
  }
  ImGui::TextUnformatted(text);
}

static void status_text_cell(ChannelStatus st, const char *text,
                             bool right_align) {
  ImGui::PushStyleColor(ImGuiCol_Text, status_color(st));
  if (right_align) {
    right_aligned_text(text);
  } else {
    ImGui::TextUnformatted(text);
  }
  ImGui::PopStyleColor();
}

/* instruments */

static bool begin_instrument_table(const char *id) {
  if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_RowBg)) {
    return false;
  }
  const float fs = ImGui::GetFontSize();
  ImGui::TableSetupColumn("channel", ImGuiTableColumnFlags_WidthFixed,
                          fs * 10.0f);
  ImGui::TableSetupColumn("bar", ImGuiTableColumnFlags_WidthStretch);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthFixed, fs * 8.0f);
  ImGui::TableSetupColumn("status", ImGuiTableColumnFlags_WidthFixed,
                          fs * 3.5f);
  return true;
}

static void group_row(const char *name) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", name);
}

/* `v` is on the range's own scale; `disp_scale` only converts it for display
 * (e.g. 100 to show a 0..1 fraction as percent). `range` may be NULL for
 * channels with no limits (no bar). */
static void instrument_row(const char *label, double v, double disp_scale,
                           const char *unit, int precision,
                           const ChannelRange *range, ChannelStatus st) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);

  ImGui::TableSetColumnIndex(1);
  if (range) {
    const bool tinted = st == CHANNEL_WARN || st == CHANNEL_ALERT;
    if (tinted) {
      ImGui::PushStyleColor(ImGuiCol_PlotHistogram, status_color(st));
    }
    ImGui::ProgressBar(fraction_of(v, *range), ImVec2(-FLT_MIN, 0.0f), "");
    if (tinted) {
      ImGui::PopStyleColor();
    }
  }

  char buf[48];
  snprintf(buf, sizeof buf, "%.*f %s", precision, v * disp_scale, unit);
  ImGui::TableSetColumnIndex(2);
  status_text_cell(st, buf, true);

  ImGui::TableSetColumnIndex(3);
  status_text_cell(st, range ? status_token(st) : "", false);
}

void instruments_panel_draw(bool *open, const ModelState *s) {
  ImGui::SetNextWindowSize(ImVec2(560.0f, 520.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_INSTRUMENTS, open)) {
    ImGui::End();
    return;
  }

  if (begin_instrument_table("instruments")) {
    group_row("ENGINE");
    instrument_row("revs per minute", s->rpm, 1.0, "rpm", 0, &CHANNEL_RANGE_RPM,
                   channel_rpm_status(s->rpm));
    instrument_row("manifold press.", s->engine.map_kpa, 1.0, "kPa", 1, NULL,
                   CHANNEL_OK);
    instrument_row("torque", s->torque_nm, 1.0, "N.m", 1, NULL, CHANNEL_OK);
    instrument_row("idle governor", s->engine.ecu.idle_throttle, 100.0, "% thr",
                   1, NULL, CHANNEL_OK);
    instrument_row("prop load", s->prop.torque_nm, 1.0, "N.m", 1, NULL,
                   CHANNEL_OK);
    instrument_row("prop thrust", s->prop.thrust_n, 1.0, "N", 0, NULL,
                   CHANNEL_OK);

    group_row("THERMAL");
    instrument_row("cyl head temp", s->thermal.cht_c, 1.0, "degC", 0,
                   &CHANNEL_RANGE_CHT,
                   channel_status_for(s->thermal.cht_c, CHANNEL_RANGE_CHT));
    instrument_row("exhaust gas temp", s->thermal.egt_c, 1.0, "degC", 0,
                   &CHANNEL_RANGE_EGT,
                   channel_status_for(s->thermal.egt_c, CHANNEL_RANGE_EGT));
    instrument_row(
        "oil temp", s->thermal.oil_temp_c, 1.0, "degC", 0,
        &CHANNEL_RANGE_OIL_TEMP,
        channel_status_for(s->thermal.oil_temp_c, CHANNEL_RANGE_OIL_TEMP));

    group_row("FUEL / LUBE");
    instrument_row(
        "oil pressure", s->lube.oil_press_kpa, 1.0, "kPa", 0,
        &CHANNEL_RANGE_OIL_PRESS,
        channel_status_for(s->lube.oil_press_kpa, CHANNEL_RANGE_OIL_PRESS));
    instrument_row(
        "fuel pressure", s->fuel.fuel_press_kpa, 1.0, "kPa", 0,
        &CHANNEL_RANGE_FUEL_PRESS,
        channel_status_for(s->fuel.fuel_press_kpa, CHANNEL_RANGE_FUEL_PRESS));
    instrument_row("fuel flow", s->fuel.fuel_flow_kgph, 1.0, "kg/h", 2, NULL,
                   CHANNEL_OK);
    instrument_row("air flow", s->fuel.air_flow_gps, 1.0, "g/s", 1, NULL,
                   CHANNEL_OK);

    group_row("ELECTRICAL");
    instrument_row("bus voltage", s->elec.bus_v, 1.0, "V", 1,
                   &CHANNEL_RANGE_BUS_V,
                   channel_status_for(s->elec.bus_v, CHANNEL_RANGE_BUS_V));
    instrument_row("alternator", s->elec.alt_current_a, 1.0, "A", 1, NULL,
                   CHANNEL_OK);
    instrument_row(
        "battery SoC", s->elec.batt_soc, 100.0, "%", 0, &CHANNEL_RANGE_BATT_SOC,
        channel_status_for(s->elec.batt_soc, CHANNEL_RANGE_BATT_SOC));

    ImGui::EndTable();
  }

  ImGui::End();
}

/*  environment  */

static void reading_row(const char *label, double v, const char *unit,
                        int precision) {
  char buf[48];
  snprintf(buf, sizeof buf, "%.*f %s", precision, v, unit);
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  right_aligned_text(buf);
}

void environment_panel_draw(bool *open, const ModelState *s) {
  ImGui::SetNextWindowSize(ImVec2(320.0f, 170.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ENVIRONMENT, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::BeginTable("environment", 2, ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 10.0f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    reading_row("outside air temp", s->env.oat_c, "degC", 1);
    reading_row("ambient press", s->env.ambient_kpa, "kPa", 1);
    reading_row("density altitude", s->env.density_alt_m, "m", 0);
    reading_row("true airspeed", s->env.airspeed_ms, "m/s", 1);
    ImGui::EndTable();
  }

  ImGui::End();
}

/*  cylinders  */

static void number_cell(int col, ChannelStatus st, int precision, double v) {
  char buf[32];
  snprintf(buf, sizeof buf, "%.*f", precision, v);
  ImGui::TableSetColumnIndex(col);
  status_text_cell(st, buf, true);
}

void cylinders_panel_draw(bool *open, const ModelState *s, int num_cyl) {
  ImGui::SetNextWindowSize(ImVec2(420.0f, 260.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_CYLINDERS, open)) {
    ImGui::End();
    return;
  }

  const int nc =
      num_cyl < 1
          ? 1
          : (num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS : num_cyl);
  double max_misfire = 0.0;

  if (ImGui::BeginTable("cylinders", 5,
                        ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_BordersInnerV)) {
    ImGui::TableSetupColumn("Cyl", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 3.0f);
    ImGui::TableSetupColumn("CHT degC");
    ImGui::TableSetupColumn("EGT degC");
    ImGui::TableSetupColumn("Lambda");
    ImGui::TableSetupColumn("Misfire %");
    ImGui::TableHeadersRow();

    for (int i = 0; i < nc; i++) {
      const CylinderState &c = s->cyl[i];
      if (c.misfire_rate > max_misfire) {
        max_misfire = c.misfire_rate;
      }
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("%d", i + 1);
      number_cell(1, channel_status_for(c.cht_c, CHANNEL_RANGE_CHT), 0,
                  c.cht_c);
      number_cell(2, channel_status_for(c.egt_c, CHANNEL_RANGE_EGT), 0,
                  c.egt_c);
      number_cell(3, channel_status_for(c.lambda, CHANNEL_RANGE_LAMBDA), 2,
                  c.lambda);
      number_cell(4, misfire_status(c.misfire_rate), 0, c.misfire_rate * 100.0);
    }
    ImGui::EndTable();
  }

  const ChannelStatus worst = misfire_status(max_misfire);
  ImGui::TextDisabled("worst misfire");
  ImGui::SameLine();
  ImGui::PushStyleColor(ImGuiCol_Text, status_color(worst));
  ImGui::Text("%.0f%%  %s", max_misfire * 100.0, status_token(worst));
  ImGui::PopStyleColor();

  ImGui::End();
}

/*  sim status  */

static const char *run_state_name(EngineRunState st) {
  switch (st) {
  case ENGINE_CRANKING:
    return "CRANKING";
  case ENGINE_RUNNING:
    return "RUNNING";
  case ENGINE_STOPPED:
  default:
    return "STOPPED";
  }
}

void sim_panel_draw(bool *open, const ModelState *s, double sim_time_s,
                    double throttle, float fps, bool sensor_mode,
                    SimClock *clock) {
  ImGui::SetNextWindowSize(ImVec2(360.0f, 300.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_SIM, open)) {
    ImGui::End();
    return;
  }

  ImGui::Text("aero engine digital twin");
  ImGui::TextDisabled("T+%07.1f s   %2.0f FPS", sim_time_s, (double)fps);
  ImGui::Separator();

  const EngineRunState rs = s->engine.run_state;
  ImGui::TextDisabled("engine");
  ImGui::SameLine();
  ImGui::PushStyleColor(
      ImGuiCol_Text,
      rs == ENGINE_CRANKING
          ? COL_CAUTION
          : (rs == ENGINE_STOPPED
                 ? ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]
                 : ImGui::GetStyle().Colors[ImGuiCol_Text]));
  ImGui::TextUnformatted(run_state_name(rs));
  ImGui::PopStyleColor();

  ImGui::TextDisabled("display feed");
  ImGui::SameLine();
  ImGui::TextUnformatted(sensor_mode ? "SENSOR (noisy)" : "MODEL (exact)");

  char overlay[16];
  snprintf(overlay, sizeof overlay, "%.0f%%", throttle * 100.0);
  ImGui::TextDisabled("throttle");
  ImGui::SameLine();
  ImGui::ProgressBar((float)throttle, ImVec2(-FLT_MIN, 0.0f), overlay);

  ImGui::Separator();
  ImGui::TextDisabled("sim clock");
  ImGui::SameLine();
  if (clock->paused) {
    ImGui::TextColored(COL_CAUTION, "PAUSED");
  } else {
    ImGui::Text("running  %.2gx", sim_clock_speed(clock));
  }
  if (ImGui::Button(clock->paused ? "Resume" : "Pause")) {
    clock->paused = !clock->paused;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!clock->paused);
  if (ImGui::Button("Step")) {
    sim_clock_request_step(clock);
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("advance %.2f s (while paused)", SIM_CLOCK_STEP_S);
  }
  for (int i = 0; i < SIM_CLOCK_NUM_SPEEDS; i++) {
    char label[16];
    snprintf(label, sizeof label, "%.2gx", SIM_CLOCK_SPEEDS[i]);
    if (i > 0) {
      ImGui::SameLine();
    }
    if (ImGui::RadioButton(label, clock->speed_idx == i)) {
      sim_clock_set_speed(clock, i);
    }
  }

  if (ImGui::CollapsingHeader("Keys", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextDisabled("UP/DN or W/S   throttle");
    ImGui::TextDisabled("PGUP/DN        altitude");
    ImGui::TextDisabled("[ ]            airspeed");
    ImGui::TextDisabled("- =            OAT offset");
    ImGui::TextDisabled("R              reset flight condition");
    ImGui::TextDisabled("I / O          start / stop engine");
    ImGui::TextDisabled("M              sensor / model feed");
    ImGui::TextDisabled("P / .          pause / step sim");
    ImGui::TextDisabled("G / L          gamepad panel / event log");
    ImGui::TextDisabled("F              fullscreen");
    ImGui::TextDisabled("SPACE          acknowledge alarms");
  }

  ImGui::End();
}
