#ifndef TELEMETRY_MONITOR_H
#define TELEMETRY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "telemetry/event_log.h"

/* Operating limits for the monitored channels */
typedef enum {
  CHANNEL_OK = 0, /* within limits */
  CHANNEL_WARN,   /* caution band */
  CHANNEL_ALERT,  /* warning band */
  CHANNEL_STALE   /* no / expired data */
} ChannelStatus;

typedef struct {
  double lo, hi;             /* scale endpoints (bar length / dial sweep) */
  double warn_lo, warn_hi;   /* caution thresholds */
  double alert_lo, alert_hi; /* warning thresholds */
  int has_warn_lo, has_warn_hi;
  int has_alert_lo, has_alert_hi;
} ChannelRange;

ChannelStatus channel_status_for(double value, ChannelRange range);

/* "NOMINAL" / "CAUTION" / "WARNING" / "NO DATA". */
const char *channel_status_word(ChannelStatus s);

/* Placeholder limits for the demo engine -- not from a spec sheet. */
extern const ChannelRange CHANNEL_RANGE_RPM;
extern const ChannelRange CHANNEL_RANGE_CHT;
extern const ChannelRange CHANNEL_RANGE_EGT;
extern const ChannelRange CHANNEL_RANGE_OIL_TEMP;
extern const ChannelRange CHANNEL_RANGE_OIL_PRESS;
extern const ChannelRange CHANNEL_RANGE_FUEL_PRESS;
extern const ChannelRange CHANNEL_RANGE_BUS_V;
extern const ChannelRange CHANNEL_RANGE_BATT_SOC;
extern const ChannelRange CHANNEL_RANGE_LAMBDA;

/* RPM status, or CHANNEL_STALE while the engine is at/below cranking speed
 * (the underspeed limit is meaningless then). */
ChannelStatus channel_rpm_status(double rpm);

/* The channels that are limit-monitored (fault log, alarm strip). */
typedef enum {
  MON_RPM = 0,
  MON_CHT,
  MON_EGT,
  MON_OIL_TEMP,
  MON_OIL_PRESS,
  MON_FUEL_PRESS,
  MON_BUS_V,
  MON_BATT_SOC,
  MON_CHANNELS
} MonitorChannel;

/* Short display caption, e.g. "OIL P". */
const char *monitor_channel_name(MonitorChannel ch);

/* Classify every monitored channel of `s` against its limits. */
void monitor_classify(const ModelState *s, ChannelStatus out[MON_CHANNELS]);

/* Remembers each channel's last status so check() logs only transitions. */
typedef struct {
  ChannelStatus prev_rpm, prev_cht, prev_egt, prev_oil_temp, prev_oil_press,
      prev_fuel_press, prev_bus_v, prev_batt_soc;
} FaultMonitor;

void fault_monitor_init(FaultMonitor *m);

/* Re-classify each monitored channel and push one EventLog record for each
 * channel*/
void fault_monitor_check(FaultMonitor *m, EventLog *log, const ModelState *s,
                         double sim_time_s);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_MONITOR_H */
