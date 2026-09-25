#include "telemetry/monitor.h"

#include <string.h>

#include "test_util.h"

static void test_status_one_sided_high(void) {
  ChannelRange r = CHANNEL_RANGE_CHT; /* warn_hi 210, alert_hi 240 */
  CHECK(channel_status_for(100.0, r) == CHANNEL_OK);
  CHECK(channel_status_for(210.0, r) == CHANNEL_WARN);
  CHECK(channel_status_for(239.9, r) == CHANNEL_WARN);
  CHECK(channel_status_for(240.0, r) == CHANNEL_ALERT);
}

static void test_status_both_sides_worst_wins(void) {
  ChannelRange r = CHANNEL_RANGE_BUS_V; /* lo 12.8/11.8, hi 14.9/15.4 */
  CHECK(channel_status_for(14.0, r) == CHANNEL_OK);
  CHECK(channel_status_for(12.8, r) == CHANNEL_WARN);
  CHECK(channel_status_for(11.8, r) == CHANNEL_ALERT);
  CHECK(channel_status_for(14.9, r) == CHANNEL_WARN);
  CHECK(channel_status_for(15.4, r) == CHANNEL_ALERT);
}

static void test_status_unset_thresholds_ignored(void) {
  ChannelRange none;
  memset(&none, 0, sizeof none);
  CHECK(channel_status_for(-1e9, none) == CHANNEL_OK);
  CHECK(channel_status_for(1e9, none) == CHANNEL_OK);
}

static void test_rpm_stale_at_or_below_cranking(void) {
  CHECK(channel_rpm_status(0.0) == CHANNEL_STALE);
  CHECK(channel_rpm_status(300.0) == CHANNEL_STALE);
  CHECK(channel_rpm_status(400.0) == CHANNEL_WARN); /* below idle warn_lo 500 */
  CHECK(channel_rpm_status(2200.0) == CHANNEL_OK);
  CHECK(channel_rpm_status(3100.0) == CHANNEL_ALERT);
}

static void test_status_words(void) {
  CHECK(strcmp(channel_status_word(CHANNEL_OK), "NOMINAL") == 0);
  CHECK(strcmp(channel_status_word(CHANNEL_WARN), "CAUTION") == 0);
  CHECK(strcmp(channel_status_word(CHANNEL_ALERT), "WARNING") == 0);
  CHECK(strcmp(channel_status_word(CHANNEL_STALE), "NO DATA") == 0);
}

static ModelState nominal_state(void) {
  ModelState s;
  memset(&s, 0, sizeof s);
  s.rpm = 2200.0;
  s.thermal.cht_c = 150.0;
  s.thermal.egt_c = 600.0;
  s.thermal.oil_temp_c = 90.0;
  s.lube.oil_press_kpa = 400.0;
  s.fuel.fuel_press_kpa = 300.0;
  s.elec.bus_v = 14.0;
  s.elec.batt_soc = 0.9;
  return s;
}

static void test_monitor_logs_only_transitions(void) {
  EventLog log;
  event_log_init(&log);
  FaultMonitor m;
  fault_monitor_init(&m);
  ModelState s = nominal_state();

  /* first pass: every channel goes NO DATA -> NOMINAL, one INFO each */
  fault_monitor_check(&m, &log, &s, 1.0);
  CHECK(event_log_count(&log) == 8);
  for (int i = 0; i < event_log_count(&log); i++) {
    CHECK(event_log_at(&log, i)->level == EVENT_INFO);
  }

  /* unchanged state: silent */
  fault_monitor_check(&m, &log, &s, 2.0);
  CHECK(event_log_count(&log) == 8);
}

static void test_monitor_escalation_and_recovery(void) {
  EventLog log;
  event_log_init(&log);
  FaultMonitor m;
  fault_monitor_init(&m);
  ModelState s = nominal_state();
  fault_monitor_check(&m, &log, &s, 1.0);
  int base = event_log_count(&log);

  s.thermal.cht_c = 220.0; /* caution */
  fault_monitor_check(&m, &log, &s, 2.0);
  CHECK(event_log_count(&log) == base + 1);
  const EventRecord *r = event_log_at(&log, base);
  CHECK(r->level == EVENT_CAUTION);
  CHECK(strcmp(r->category, "THERM") == 0);
  CHECK(strstr(r->message, "NOMINAL -> CAUTION") != NULL);
  CHECK_NEAR(r->sim_time_s, 2.0, 1e-9);

  s.thermal.cht_c = 245.0; /* warning */
  fault_monitor_check(&m, &log, &s, 3.0);
  CHECK(event_log_count(&log) == base + 2);
  CHECK(event_log_at(&log, base + 1)->level == EVENT_WARNING);

  s.thermal.cht_c = 150.0; /* recovered */
  fault_monitor_check(&m, &log, &s, 4.0);
  CHECK(event_log_count(&log) == base + 3);
  CHECK(event_log_at(&log, base + 2)->level == EVENT_INFO);
  CHECK(strstr(event_log_at(&log, base + 2)->message, "WARNING -> NOMINAL") !=
        NULL);
}

static const TestCase cases[] = {
    {"status one-sided high", test_status_one_sided_high},
    {"status both sides, worst wins", test_status_both_sides_worst_wins},
    {"status unset thresholds ignored", test_status_unset_thresholds_ignored},
    {"rpm stale at/below cranking", test_rpm_stale_at_or_below_cranking},
    {"status words", test_status_words},
    {"monitor logs only transitions", test_monitor_logs_only_transitions},
    {"monitor escalation and recovery", test_monitor_escalation_and_recovery},
};

RUN_TESTS(cases)
