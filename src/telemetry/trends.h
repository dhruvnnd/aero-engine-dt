#ifndef TELEMETRY_TRENDS_H
#define TELEMETRY_TRENDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "util/history.h"

/*
 * Rolling time-series of the trended channels. Each History points into the
 * matching *_buf array below */

#define TRENDS_CAP 300

typedef struct {
  double rpm_buf[TRENDS_CAP];
  double cht_buf[TRENDS_CAP];
  double egt_buf[TRENDS_CAP];
  double oil_press_buf[TRENDS_CAP];
  double batt_pct_buf[TRENDS_CAP];

  History rpm;       /* rpm */
  History cht;       /* degC */
  History egt;       /* degC */
  History oil_press; /* kPa */
  History batt_pct;  /* battery state of charge, percent */
} Trends;

void trends_init(Trends *t);

/* Append the current readings to every series. */
void trends_sample(Trends *t, const ModelState *s);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_TRENDS_H */
