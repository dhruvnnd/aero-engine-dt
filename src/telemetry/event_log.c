#include "telemetry/event_log.h"

#include <stdarg.h>
#include <stdio.h>

void event_log_init(EventLog *log) {
  log->count = 0;
  log->head = 0;
  log->total = 0;
}

void event_log_push(EventLog *log, double sim_time_s, EventLevel level,
                    const char *category, const char *fmt, ...) {
  int slot;
  if (log->count < EVENT_LOG_CAP) {
    slot = (log->head + log->count) % EVENT_LOG_CAP;
    log->count++;
  } else {
    /* full: overwrite the oldest, then advance head so it stays oldest */
    slot = log->head;
    log->head = (log->head + 1) % EVENT_LOG_CAP;
  }

  EventRecord *rec = &log->buf[slot];
  rec->sim_time_s = sim_time_s;
  rec->level = level;

  if (category) {
    snprintf(rec->category, EVENT_CATEGORY_LEN, "%s", category);
  } else {
    rec->category[0] = '\0';
  }

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(rec->message, EVENT_MESSAGE_LEN, fmt, ap);
  va_end(ap);

  log->total++;
}

int event_log_count(const EventLog *log) { return log->count; }

const EventRecord *event_log_at(const EventLog *log, int i) {
  if (i < 0 || i >= log->count) {
    return NULL;
  }
  return &log->buf[(log->head + i) % EVENT_LOG_CAP];
}
