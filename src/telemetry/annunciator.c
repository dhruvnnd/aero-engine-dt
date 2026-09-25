#include "telemetry/annunciator.h"

/* CHANNEL_STALE ("no data") is not an alarm. */
static int severity(ChannelStatus s) {
  switch (s) {
  case CHANNEL_WARN:
    return 1;
  case CHANNEL_ALERT:
    return 2;
  case CHANNEL_OK:
  case CHANNEL_STALE:
  default:
    return 0;
  }
}

void annunciator_init(Annunciator *a) {
  for (int i = 0; i < MON_CHANNELS; i++) {
    a->status[i] = CHANNEL_STALE;
    a->acked_sev[i] = 0;
  }
}

void annunciator_update(Annunciator *a, const ModelState *s) {
  monitor_classify(s, a->status);
  for (int i = 0; i < MON_CHANNELS; i++) {
    const int sev = severity(a->status[i]);
    if (sev < a->acked_sev[i]) {
      a->acked_sev[i] = sev; /* improved or recovered: a repeat is a new alarm */
    }
  }
}

void annunciator_acknowledge(Annunciator *a) {
  for (int i = 0; i < MON_CHANNELS; i++) {
    a->acked_sev[i] = severity(a->status[i]);
  }
}

ChannelStatus annunciator_status(const Annunciator *a, MonitorChannel ch) {
  return a->status[ch];
}

int annunciator_channel_unacked(const Annunciator *a, MonitorChannel ch) {
  const int sev = severity(a->status[ch]);
  return sev > 0 && sev > a->acked_sev[ch];
}

int annunciator_active(const Annunciator *a, ChannelStatus level) {
  int n = 0;
  for (int i = 0; i < MON_CHANNELS; i++) {
    if (a->status[i] == level) {
      n++;
    }
  }
  return n;
}

int annunciator_unacked(const Annunciator *a, ChannelStatus level) {
  int n = 0;
  for (int i = 0; i < MON_CHANNELS; i++) {
    if (a->status[i] == level && annunciator_channel_unacked(a, (MonitorChannel)i)) {
      n++;
    }
  }
  return n;
}
