#include "ui/dashboard.h"

#include "physics/engine_model.h"
#include "ui/ui_draw.h"
#include "ui/ui_layout.h"
#include "ui/ui_widgets.h"

static const ChannelRange RANGE_AUTO = {0};

void dashboard_init(Dashboard *d) {
  d->theme = ui_theme_default();
  ui_history_init(&d->rpm_hist, d->rpm_buf, DASHBOARD_TREND_CAP);
  ui_history_init(&d->cht_hist, d->cht_buf, DASHBOARD_TREND_CAP);
  ui_history_init(&d->egt_hist, d->egt_buf, DASHBOARD_TREND_CAP);
  ui_history_init(&d->oilp_hist, d->oilp_buf, DASHBOARD_TREND_CAP);
  ui_history_init(&d->batt_hist, d->batt_buf, DASHBOARD_TREND_CAP);
}

void dashboard_sample(Dashboard *d, const ModelState *s) {
  ui_history_push(&d->rpm_hist, s->rpm);
  ui_history_push(&d->cht_hist, s->thermal.cht_c);
  ui_history_push(&d->egt_hist, s->thermal.egt_c);
  ui_history_push(&d->oilp_hist, s->lube.oil_press_kpa);
  ui_history_push(&d->batt_hist, s->elec.batt_soc * 100.0);
}

void dashboard_draw(Dashboard *d, SDL_Renderer *r, float w, float h,
                    const ModelState *s, int num_cyl, double throttle,
                    double sim_time_s, float fps, int sensor_mode) {
  const UiTheme *th = &d->theme;

  ui_fill(r, (UiRect){0.0f, 0.0f, w, h}, th->bg);
  UiRect screen = ui_rect_inset((UiRect){0.0f, 0.0f, w, h}, 8.0f, 8.0f);

  UiRect rest;
  UiRect header = ui_split_top(screen, 14.0f, 6.0f, &rest);
  ui_text(r, header.x, header.y, th->text_bright,
          "aero engine digital twin (%d)   %s", SDL_GetVersion(),
          sensor_mode ? "[sensor]" : "[model]");
  ui_text_right(r, header.x + header.w, header.y, th->text_dim,
                "T+%07.1fs   THR %3.0f%%   %2.0f FPS", sim_time_s,
                throttle * 100.0, (double)fps);
  ui_hline(r, screen.x, header.y + 12.0f, screen.w, th->frame);

  UiRect body;
  UiRect footer = ui_split_bottom(rest, 20.0f, 6.0f, &body);
  ui_text(r, footer.x, footer.y, th->text_dim,
          "UP/DN or W/S  throttle    PGUP/DN  altitude    [ ]  airspeed    "
          "-/=  OAT");
  ui_text(r, footer.x, footer.y + UI_GLYPH_H + 2.0f, th->text_dim,
          "R  reset env    I  start engine    O  stop engine    "
          "M  sensor / model    G  gamepad panel    L  event log    "
          "F  fullscreen");

  UiRect right;
  UiRect left = ui_split_left_frac(body, 0.42f, 8.0f, &right);

  /* left column: secondary readouts (fixed, text), the round engine gauges
   * (fill the middle), per-cylinder bars pinned to the bottom. */
  UiRect leftrest;
  UiRect kpibox = ui_split_top(left, 104.0f, 8.0f, &leftrest);
  UiRect belowenv;
  UiRect envbox = ui_split_top(leftrest, 74.0f, 8.0f, &belowenv);
  UiRect gaugebox;
  UiRect cylbox = ui_split_bottom(belowenv, 255.0f, 8.0f, &gaugebox);

  UiRect kpi = ui_panel(r, kpibox, th, "secondary");
  UiGrid kg = ui_grid(kpi, 2, 2, 6.0f);
  ui_stat_tile(r, ui_grid_at(kg, 0), th, "MAP", s->engine.map_kpa, "kPa", 1);
  ui_stat_tile(r, ui_grid_at(kg, 1), th, "TORQUE", s->torque_nm, "N.m", 1);
  ui_stat_tile(r, ui_grid_at(kg, 2), th, "FUEL FLOW", s->fuel.fuel_flow_kgph,
               "kg/h", 2);
  ui_stat_tile(r, ui_grid_at(kg, 3), th, "FUEL PRESS", s->fuel.fuel_press_kpa,
               "kPa", 0);

  /* Flight condition */
  UiRect ev = ui_panel(r, envbox, th, "environment");
  UiStack es = ui_stack(ev, 2.0f);
  ui_reading_row(r, ui_stack_row(&es, th->row_h), th, "outside air temp",
                 s->env.oat_c, "degC", 1, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&es, th->row_h), th, "ambient press",
                 s->env.ambient_kpa, "kPa", 1, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&es, th->row_h), th, "density altitude",
                 s->env.density_alt_m, "m", 0, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&es, th->row_h), th, "true airspeed",
                 s->env.airspeed_ms, "m/s", 1, CHANNEL_OK);

  UiRect g = ui_panel(r, gaugebox, th, "gauges");
  UiGrid gg = ui_grid(g, 2, 2, 8.0f);
  ui_dial_gauge(r, ui_grid_at(gg, 0), th, "RPM", s->rpm, "rpm", 0, CHANNEL_RANGE_RPM);
  ui_dial_gauge(r, ui_grid_at(gg, 1), th, "CHT", s->thermal.cht_c, "degC", 0,
                CHANNEL_RANGE_CHT);
  ui_dial_gauge(r, ui_grid_at(gg, 2), th, "EGT", s->thermal.egt_c, "degC", 0,
                CHANNEL_RANGE_EGT);
  ui_dial_gauge(r, ui_grid_at(gg, 3), th, "OIL", s->thermal.oil_temp_c, "degC",
                0, CHANNEL_RANGE_OIL_TEMP);

  /* right column: trends fill the upper part, the subsystem list the rest */
  UiRect subbox;
  UiRect trends = ui_split_top_frac(right, 0.6f, 8.0f, &subbox);

  UiRect tr = ui_panel(r, trends, th, "trends");
  UiGrid tg = ui_grid(tr, 1, 5, 6.0f);
  ui_sparkline(r, ui_grid_at(tg, 0), th, "RPM", "rpm", 0, &d->rpm_hist,
               CHANNEL_RANGE_RPM);
  ui_sparkline(r, ui_grid_at(tg, 1), th, "CHT", "degC", 0, &d->cht_hist,
               RANGE_AUTO);
  ui_sparkline(r, ui_grid_at(tg, 2), th, "EGT", "degC", 0, &d->egt_hist,
               RANGE_AUTO);
  ui_sparkline(r, ui_grid_at(tg, 3), th, "OIL P", "kPa", 0, &d->oilp_hist,
               CHANNEL_RANGE_OIL_PRESS);
  ui_sparkline(r, ui_grid_at(tg, 4), th, "BATT", "%", 0, &d->batt_hist,
               RANGE_AUTO);

  /* left column bottom: per-cylinder comparison bars */
  int nc = num_cyl < 1 ? 1
                       : (num_cyl > ENGINE_MAX_CYLINDERS ? ENGINE_MAX_CYLINDERS
                                                         : num_cyl);
  double cht[ENGINE_MAX_CYLINDERS];
  double egt[ENGINE_MAX_CYLINDERS];
  double lam[ENGINE_MAX_CYLINDERS];
  double max_misfire = 0.0;
  for (int i = 0; i < nc; i++) {
    cht[i] = s->cyl[i].cht_c;
    egt[i] = s->cyl[i].egt_c;
    lam[i] = s->cyl[i].lambda;
    if (s->cyl[i].misfire_rate > max_misfire) {
      max_misfire = s->cyl[i].misfire_rate;
    }
  }
  UiRect cy = ui_panel(r, cylbox, th, "cylinders");
  UiGrid cg = ui_grid(cy, 1, 3, 6.0f);
  ui_bar_series(r, ui_grid_at(cg, 0), th, "CHT", cht, NULL, nc, "degC", 0,
                CHANNEL_RANGE_CHT);
  ui_bar_series(r, ui_grid_at(cg, 1), th, "EGT", egt, NULL, nc, "degC", 0,
                CHANNEL_RANGE_EGT);
  ui_bar_series(r, ui_grid_at(cg, 2), th, "MIX", lam, NULL, nc, "L", 2,
                CHANNEL_RANGE_LAMBDA);

  UiRect stp = ui_panel(r, subbox, th, "subsystems");
  UiStack ss = ui_stack(stp, 2.0f);
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "revs per minute", s->rpm,
                 "rpm", 0,
                 channel_rpm_status(s->rpm));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "cylinder head temp.",
                 s->thermal.cht_c, "degC", 0,
                 channel_status_for(s->thermal.cht_c, CHANNEL_RANGE_CHT));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "exhaust gas temp.",
                 s->thermal.egt_c, "degC", 0,
                 channel_status_for(s->thermal.egt_c, CHANNEL_RANGE_EGT));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "oil temp.",
                 s->thermal.oil_temp_c, "degC", 0,
                 channel_status_for(s->thermal.oil_temp_c, CHANNEL_RANGE_OIL_TEMP));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "oil pressure",
                 s->lube.oil_press_kpa, "kPa", 0,
                 channel_status_for(s->lube.oil_press_kpa, CHANNEL_RANGE_OIL_PRESS));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "fuel pressure",
                 s->fuel.fuel_press_kpa, "kPa", 0,
                 channel_status_for(s->fuel.fuel_press_kpa, CHANNEL_RANGE_FUEL_PRESS));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "fuel flow",
                 s->fuel.fuel_flow_kgph, "kg/h", 2, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "air flow",
                 s->fuel.air_flow_gps, "g/s", 1, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "misfire (worst cyl)",
                 max_misfire * 100.0, "%", 0,
                 max_misfire > 0.5   ? CHANNEL_ALERT
                 : max_misfire > 0.0 ? CHANNEL_WARN
                                     : CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "bus voltage",
                 s->elec.bus_v, "V", 1,
                 channel_status_for(s->elec.bus_v, CHANNEL_RANGE_BUS_V));
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "alternator",
                 s->elec.alt_current_a, "A", 1, CHANNEL_OK);
  ui_reading_row(r, ui_stack_row(&ss, th->row_h), th, "battery SoC",
                 s->elec.batt_soc * 100.0, "%", 0,
                 channel_status_for(s->elec.batt_soc, CHANNEL_RANGE_BATT_SOC));
}
