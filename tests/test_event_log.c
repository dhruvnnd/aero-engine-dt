#include "telemetry/event_log.h"

#include <string.h>

#include "test_util.h"

static void test_empty(void) {
  EventLog log;
  event_log_init(&log);

  CHECK(event_log_count(&log) == 0);
  CHECK(event_log_at(&log, 0) == NULL);
  CHECK(event_log_at(&log, -1) == NULL);
}

static void test_push_and_read_order(void) {
  EventLog log;
  event_log_init(&log);

  event_log_push(&log, 1.0, EVENT_INFO, "MODE", "sensor -> model");
  event_log_push(&log, 2.5, EVENT_CAUTION, "THERM", "CHT %d C", 215);

  CHECK(event_log_count(&log) == 2);

  const EventRecord *r0 = event_log_at(&log, 0); /* oldest */
  const EventRecord *r1 = event_log_at(&log, 1); /* newest */
  CHECK(r0 != NULL && r1 != NULL);
  CHECK_NEAR(r0->sim_time_s, 1.0, 1e-9);
  CHECK(r0->level == EVENT_INFO);
  CHECK(strcmp(r0->category, "MODE") == 0);
  CHECK(strcmp(r0->message, "sensor -> model") == 0);

  CHECK_NEAR(r1->sim_time_s, 2.5, 1e-9);
  CHECK(r1->level == EVENT_CAUTION);
  CHECK(strcmp(r1->message, "CHT 215 C") == 0);

  CHECK(event_log_at(&log, 2) == NULL);
}

static void test_truncation(void) {
  EventLog log;
  event_log_init(&log);

  /* category longer than the field: must not overflow, just truncate */
  event_log_push(&log, 0.0, EVENT_WARNING, "WAY_TOO_LONG_A_CATEGORY_NAME",
                "%s", "also a fairly long message body just to be sure");
  const EventRecord *r = event_log_at(&log, 0);
  CHECK(r != NULL);
  CHECK(strlen(r->category) == EVENT_CATEGORY_LEN - 1);
  CHECK(strlen(r->message) < EVENT_MESSAGE_LEN);
}

static void test_ring_eviction(void) {
  EventLog log;
  event_log_init(&log);

  /* push one more than capacity; the oldest should fall off, order intact */
  for (int i = 0; i < EVENT_LOG_CAP + 3; i++) {
    event_log_push(&log, (double)i, EVENT_INFO, "SEQ", "n=%d", i);
  }

  CHECK(event_log_count(&log) == EVENT_LOG_CAP);
  const EventRecord *oldest = event_log_at(&log, 0);
  const EventRecord *newest = event_log_at(&log, EVENT_LOG_CAP - 1);
  CHECK(oldest != NULL && newest != NULL);
  CHECK_NEAR(oldest->sim_time_s, 3.0, 1e-9); /* first 3 evicted: 0,1,2 */
  CHECK_NEAR(newest->sim_time_s, (double)(EVENT_LOG_CAP + 2), 1e-9);
}

static const TestCase kCases[] = {
    {"empty", test_empty},
    {"push_and_read_order", test_push_and_read_order},
    {"truncation", test_truncation},
    {"ring_eviction", test_ring_eviction},
};

RUN_TESTS(kCases)
