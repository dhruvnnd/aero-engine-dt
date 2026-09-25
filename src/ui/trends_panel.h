#ifndef UI_TRENDS_PANEL_H
#define UI_TRENDS_PANEL_H

#include "telemetry/trends.h"

/* Dockable ImPlot window: one interactive line plot per trended channel,
 * stacked with a shared time axis (seconds relative to now, so 0 is the newest
 * sample). Caution / warning limits from telemetry/monitor are drawn as
 * horizontal lines. `sample_period_s` is the cadence trends_sample() runs at.
 * `open` is cleared when the user closes the window. */
void trends_panel_draw(bool *open, const Trends *t, double sample_period_s);

#endif /* UI_TRENDS_PANEL_H */
