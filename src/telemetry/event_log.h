#ifndef TELEMETRY_EVENT_LOG_H
#define TELEMETRY_EVENT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed-capacity ring buffer of application/sim events
 */

#define EVENT_LOG_CAP 512
#define EVENT_CATEGORY_LEN 16
#define EVENT_MESSAGE_LEN 96

typedef enum {
  EVENT_INFO = 0, /* routine: mode/config/connect changes */
  EVENT_CAUTION,  /* a monitored channel entered its caution band */
  EVENT_WARNING   /* a monitored channel entered its warning band */
} EventLevel;

typedef struct {
  double sim_time_s;
  EventLevel level;
  char category[EVENT_CATEGORY_LEN];
  char message[EVENT_MESSAGE_LEN];
} EventRecord;

typedef struct {
  EventRecord buf[EVENT_LOG_CAP];
  int count;  /* live records, 0..EVENT_LOG_CAP */
  int head;   /* index of the oldest record */
  long total; /* lifetime count pushed, including evicted ones */
} EventLog;

void event_log_init(EventLog *log);

/* Appends one record, evicting the oldest once full. `category` is copied
 * and truncated to EVENT_CATEGORY_LEN-1 chars; `message` is formatted
 * printf-style and truncated to EVENT_MESSAGE_LEN-1. */
void event_log_push(EventLog *log, double sim_time_s, EventLevel level,
                    const char *category, const char *fmt, ...);

int event_log_count(const EventLog *log);

/* Record by age index: 0 = oldest retained, count-1 = newest. NULL if out of
 * range. */
const EventRecord *event_log_at(const EventLog *log, int i);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_EVENT_LOG_H */
