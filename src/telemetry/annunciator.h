#ifndef TELEMETRY_ANNUNCIATOR_H
#define TELEMETRY_ANNUNCIATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/monitor.h"

/*
 * Master caution / warning bookkeeping, cockpit-style. Each monitored channel
 * is in one of three severities: none (nominal or no data), caution, warning.
 * An alarm is "unacknowledged" until acknowledge() is called; it stays lit but
 * stops demanding attention. An acknowledged channel that gets worse (caution
 * -> warning) is re-armed; one that recovers is forgotten, so a later fault is
 * unacknowledged again.
 */

typedef struct {
  ChannelStatus status[MON_CHANNELS];
  int acked_sev[MON_CHANNELS]; /* severity acknowledged so far, 0..2 */
} Annunciator;

void annunciator_init(Annunciator *a);

/* Re-classify every channel of `s` and update acknowledgement state. Call at a
 * fixed cadence alongside fault_monitor_check(). */
void annunciator_update(Annunciator *a, const ModelState *s);

/* Acknowledge everything currently active. */
void annunciator_acknowledge(Annunciator *a);

/* Status of one channel as last classified. */
ChannelStatus annunciator_status(const Annunciator *a, MonitorChannel ch);

/* True if `ch` is in caution/warning and not yet acknowledged at that level. */
int annunciator_channel_unacked(const Annunciator *a, MonitorChannel ch);

/* Number of channels currently at `level` (CHANNEL_WARN = caution,
 * CHANNEL_ALERT = warning). */
int annunciator_active(const Annunciator *a, ChannelStatus level);

/* Number of those at `level` that are unacknowledged. */
int annunciator_unacked(const Annunciator *a, ChannelStatus level);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_ANNUNCIATOR_H */
