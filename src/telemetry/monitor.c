#include "telemetry/monitor.h"

/* lo/hi frame the gauge track; *_hi thresholds trip on the way up, *_lo on the
 * way down. */

const ChannelRange CHANNEL_RANGE_RPM = {.lo = 0.0,
                                        .hi = 3200.0,
                                        .warn_lo = 500.0,  /* idle underspeed */
                                        .alert_lo = 300.0, /* near flame-out */
                                        .warn_hi = 2900.0,
                                        .alert_hi = 3050.0,
                                        .has_warn_lo = 1,
                                        .has_alert_lo = 1,
                                        .has_warn_hi = 1,
                                        .has_alert_hi = 1};
const ChannelRange CHANNEL_RANGE_CHT = {.lo = 0.0,
                                        .hi = 260.0,
                                        .warn_hi = 210.0,
                                        .alert_hi = 240.0,
                                        .has_warn_hi = 1,
                                        .has_alert_hi = 1};
const ChannelRange CHANNEL_RANGE_EGT = {.lo = 0.0,
                                        .hi = 900.0,
                                        .warn_hi = 780.0,
                                        .alert_hi = 850.0,
                                        .has_warn_hi = 1,
                                        .has_alert_hi = 1};
const ChannelRange CHANNEL_RANGE_OIL_TEMP = {.lo = 0.0,
                                             .hi = 140.0,
                                             .warn_hi = 110.0,
                                             .alert_hi = 125.0,
                                             .has_warn_hi = 1,
                                             .has_alert_hi = 1};

/* Fuel supply pressure: both-sided -- a drop below minimum is the real fault */
const ChannelRange CHANNEL_RANGE_FUEL_PRESS = {.lo = 0.0,
                                               .hi = 400.0,
                                               .warn_lo = 250.0,
                                               .alert_lo = 200.0,
                                               .warn_hi = 360.0,
                                               .has_warn_lo = 1,
                                               .has_alert_lo = 1,
                                               .has_warn_hi = 1};
const ChannelRange CHANNEL_RANGE_OIL_PRESS = {.lo = 0.0,
                                              .hi = 800.0,
                                              .warn_lo = 150.0,
                                              .alert_lo = 120.0,
                                              .warn_hi = 600.0,
                                              .alert_hi = 620.0,
                                              .has_warn_lo = 1,
                                              .has_alert_lo = 1,
                                              .has_warn_hi = 1};
/* Main bus voltage: both-sided. A sag means the alternator isn't carrying
 * the load; a spike means the regulator is overcharging. */
const ChannelRange CHANNEL_RANGE_BUS_V = {.lo = 10.0,
                                          .hi = 16.0,
                                          .warn_lo = 12.8,
                                          .alert_lo = 11.8,
                                          .warn_hi = 14.9,
                                          .alert_hi = 15.4,
                                          .has_warn_lo = 1,
                                          .has_alert_lo = 1,
                                          .has_warn_hi = 1,
                                          .has_alert_hi = 1};
/* Battery state of charge: one-sided -- how much reserve is left. */
const ChannelRange CHANNEL_RANGE_BATT_SOC = {.lo = 0.0,
                                             .hi = 1.0,
                                             .warn_lo = 0.35,
                                             .alert_lo = 0.18,
                                             .has_warn_lo = 1,
                                             .has_alert_lo = 1};

/* Air-fuel equivalence ratio, 1.0 = stoichiometric. Lean (>1) or rich (<1)
 * both flag; ~1.55 lean is the misfire edge. */
const ChannelRange CHANNEL_RANGE_LAMBDA = {.lo = 0.70,
                                           .hi = 1.70,
                                           .warn_lo = 0.90,
                                           .alert_lo = 0.80,
                                           .warn_hi = 1.10,
                                           .alert_hi = 1.55,
                                           .has_warn_lo = 1,
                                           .has_alert_lo = 1,
                                           .has_warn_hi = 1,
                                           .has_alert_hi = 1};

ChannelStatus channel_status_for(double value, ChannelRange range) {
  if ((range.has_alert_lo && value <= range.alert_lo) ||
      (range.has_alert_hi && value >= range.alert_hi)) {
    return CHANNEL_ALERT;
  }
  if ((range.has_warn_lo && value <= range.warn_lo) ||
      (range.has_warn_hi && value >= range.warn_hi)) {
    return CHANNEL_WARN;
  }
  return CHANNEL_OK;
}

const char *channel_status_word(ChannelStatus s) {
  switch (s) {
  case CHANNEL_OK:
    return "NOMINAL";
  case CHANNEL_WARN:
    return "CAUTION";
  case CHANNEL_ALERT:
    return "WARNING";
  case CHANNEL_STALE:
  default:
    return "NO DATA";
  }
}

ChannelStatus channel_rpm_status(double rpm) {
  return rpm > 300.0 ? channel_status_for(rpm, CHANNEL_RANGE_RPM)
                     : CHANNEL_STALE;
}

const char *monitor_channel_name(MonitorChannel ch) {
  switch (ch) {
  case MON_RPM:
    return "RPM";
  case MON_CHT:
    return "CHT";
  case MON_EGT:
    return "EGT";
  case MON_OIL_TEMP:
    return "OIL T";
  case MON_OIL_PRESS:
    return "OIL P";
  case MON_FUEL_PRESS:
    return "FUEL P";
  case MON_BUS_V:
    return "BUS V";
  case MON_BATT_SOC:
    return "BATT";
  default:
    return "?";
  }
}

void monitor_classify(const ModelState *s, ChannelStatus out[MON_CHANNELS]) {
  out[MON_RPM] = channel_rpm_status(s->rpm);
  out[MON_CHT] = channel_status_for(s->thermal.cht_c, CHANNEL_RANGE_CHT);
  out[MON_EGT] = channel_status_for(s->thermal.egt_c, CHANNEL_RANGE_EGT);
  out[MON_OIL_TEMP] =
      channel_status_for(s->thermal.oil_temp_c, CHANNEL_RANGE_OIL_TEMP);
  out[MON_OIL_PRESS] =
      channel_status_for(s->lube.oil_press_kpa, CHANNEL_RANGE_OIL_PRESS);
  out[MON_FUEL_PRESS] =
      channel_status_for(s->fuel.fuel_press_kpa, CHANNEL_RANGE_FUEL_PRESS);
  out[MON_BUS_V] = channel_status_for(s->elec.bus_v, CHANNEL_RANGE_BUS_V);
  out[MON_BATT_SOC] =
      channel_status_for(s->elec.batt_soc, CHANNEL_RANGE_BATT_SOC);
}

void fault_monitor_init(FaultMonitor *m) {
  m->prev_rpm = CHANNEL_STALE;
  m->prev_cht = CHANNEL_STALE;
  m->prev_egt = CHANNEL_STALE;
  m->prev_oil_temp = CHANNEL_STALE;
  m->prev_oil_press = CHANNEL_STALE;
  m->prev_fuel_press = CHANNEL_STALE;
  m->prev_bus_v = CHANNEL_STALE;
  m->prev_batt_soc = CHANNEL_STALE;
}

static void check_channel(EventLog *log, double sim_time_s, ChannelStatus *prev,
                          ChannelStatus cur, const char *name,
                          const char *category, double value, const char *unit,
                          int precision) {
  if (cur == *prev) {
    return;
  }
  EventLevel lvl = (cur == CHANNEL_ALERT)  ? EVENT_WARNING
                   : (cur == CHANNEL_WARN) ? EVENT_CAUTION
                                           : EVENT_INFO;
  event_log_push(log, sim_time_s, lvl, category, "%s %s -> %s (%.*f %s)", name,
                 channel_status_word(*prev), channel_status_word(cur),
                 precision, value, unit);
  *prev = cur;
}

void fault_monitor_check(FaultMonitor *m, EventLog *log, const ModelState *s,
                         double sim_time_s) {
  ChannelStatus st[MON_CHANNELS];
  monitor_classify(s, st);
  check_channel(log, sim_time_s, &m->prev_rpm, st[MON_RPM], "revs per minute",
                "RPM", s->rpm, "rpm", 0);
  check_channel(log, sim_time_s, &m->prev_cht, st[MON_CHT],
                "cylinder head temp", "THERM", s->thermal.cht_c, "degC", 0);
  check_channel(log, sim_time_s, &m->prev_egt, st[MON_EGT], "exhaust gas temp",
                "THERM", s->thermal.egt_c, "degC", 0);
  check_channel(log, sim_time_s, &m->prev_oil_temp, st[MON_OIL_TEMP],
                "oil temp", "THERM", s->thermal.oil_temp_c, "degC", 0);
  check_channel(log, sim_time_s, &m->prev_oil_press, st[MON_OIL_PRESS],
                "oil pressure", "LUBE", s->lube.oil_press_kpa, "kPa", 0);
  check_channel(log, sim_time_s, &m->prev_fuel_press, st[MON_FUEL_PRESS],
                "fuel pressure", "FUEL", s->fuel.fuel_press_kpa, "kPa", 0);
  check_channel(log, sim_time_s, &m->prev_bus_v, st[MON_BUS_V], "bus voltage",
                "ELEC", s->elec.bus_v, "V", 1);
  check_channel(log, sim_time_s, &m->prev_batt_soc, st[MON_BATT_SOC],
                "battery SoC", "ELEC", s->elec.batt_soc * 100.0, "%", 0);
}
