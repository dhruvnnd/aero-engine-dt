#include "telemetry/annunciator.h"

#include <string.h>

#include "test_util.h"

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

static void test_nominal_is_quiet(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  annunciator_update(&a, &s);

  CHECK(annunciator_active(&a, CHANNEL_WARN) == 0);
  CHECK(annunciator_active(&a, CHANNEL_ALERT) == 0);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 0);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 0);
}

static void test_no_data_is_not_an_alarm(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.rpm = 0.0; /* engine stopped: RPM is "no data", not an underspeed alarm */
  annunciator_update(&a, &s);

  CHECK(annunciator_status(&a, MON_RPM) == CHANNEL_STALE);
  CHECK(annunciator_active(&a, CHANNEL_WARN) == 0);
  CHECK(annunciator_active(&a, CHANNEL_ALERT) == 0);
  CHECK(!annunciator_channel_unacked(&a, MON_RPM));
}

static void test_fault_counts_and_ack(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.thermal.cht_c = 220.0;     /* caution */
  s.thermal.egt_c = 860.0;     /* warning */
  annunciator_update(&a, &s);

  CHECK(annunciator_active(&a, CHANNEL_WARN) == 1);
  CHECK(annunciator_active(&a, CHANNEL_ALERT) == 1);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 1);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 1);
  CHECK(annunciator_channel_unacked(&a, MON_CHT));

  annunciator_acknowledge(&a);
  CHECK(annunciator_active(&a, CHANNEL_WARN) == 1); /* still lit */
  CHECK(annunciator_active(&a, CHANNEL_ALERT) == 1);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 0);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 0);

  annunciator_update(&a, &s); /* condition persists: stays acknowledged */
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 0);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 0);
}

static void test_escalation_rearms(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.thermal.cht_c = 220.0; /* caution */
  annunciator_update(&a, &s);
  annunciator_acknowledge(&a);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 0);

  s.thermal.cht_c = 245.0; /* caution -> warning */
  annunciator_update(&a, &s);
  CHECK(annunciator_status(&a, MON_CHT) == CHANNEL_ALERT);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 1); /* re-armed */
}

static void test_new_channel_is_unacked(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.thermal.cht_c = 220.0;
  annunciator_update(&a, &s);
  annunciator_acknowledge(&a);

  s.elec.bus_v = 15.0; /* a different channel goes into caution */
  annunciator_update(&a, &s);
  CHECK(annunciator_active(&a, CHANNEL_WARN) == 2);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 1);
  CHECK(annunciator_channel_unacked(&a, MON_BUS_V));
  CHECK(!annunciator_channel_unacked(&a, MON_CHT));
}

static void test_recovery_forgets_ack(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.thermal.cht_c = 245.0;
  annunciator_update(&a, &s);
  annunciator_acknowledge(&a);

  s.thermal.cht_c = 150.0; /* recovered */
  annunciator_update(&a, &s);
  CHECK(annunciator_active(&a, CHANNEL_ALERT) == 0);

  s.thermal.cht_c = 245.0; /* the same fault again is a new alarm */
  annunciator_update(&a, &s);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 1);
}

static void test_deescalation_keeps_ack_then_rearms(void) {
  Annunciator a;
  annunciator_init(&a);
  ModelState s = nominal_state();
  s.thermal.cht_c = 245.0; /* warning */
  annunciator_update(&a, &s);
  annunciator_acknowledge(&a);

  s.thermal.cht_c = 220.0; /* warning -> caution: already acknowledged */
  annunciator_update(&a, &s);
  CHECK(annunciator_active(&a, CHANNEL_WARN) == 1);
  CHECK(annunciator_unacked(&a, CHANNEL_WARN) == 0);

  s.thermal.cht_c = 245.0; /* and back up: re-armed */
  annunciator_update(&a, &s);
  CHECK(annunciator_unacked(&a, CHANNEL_ALERT) == 1);
}

static void test_monitor_classify_matches_status_for(void) {
  ModelState s = nominal_state();
  s.thermal.egt_c = 800.0;
  ChannelStatus st[MON_CHANNELS];
  monitor_classify(&s, st);
  CHECK(st[MON_RPM] == CHANNEL_OK);
  CHECK(st[MON_EGT] == CHANNEL_WARN);
  CHECK(st[MON_CHT] == CHANNEL_OK);
  CHECK(strcmp(monitor_channel_name(MON_OIL_PRESS), "OIL P") == 0);
}

static const TestCase cases[] = {
    {"nominal is quiet", test_nominal_is_quiet},
    {"no data is not an alarm", test_no_data_is_not_an_alarm},
    {"fault counts and ack", test_fault_counts_and_ack},
    {"escalation re-arms", test_escalation_rearms},
    {"new channel is unacked", test_new_channel_is_unacked},
    {"recovery forgets ack", test_recovery_forgets_ack},
    {"de-escalation keeps ack, re-arms", test_deescalation_keeps_ack_then_rearms},
    {"monitor_classify", test_monitor_classify_matches_status_for},
};

RUN_TESTS(cases)
