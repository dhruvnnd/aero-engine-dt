#ifndef UI_TRENDS_PANEL_H
#define UI_TRENDS_PANEL_H

#include "telemetry/trends.h"

/* Dockable ImPlot window: one interactive plot per trended channel, stacked
 * with a shared time axis (seconds relative to now, so 0 is the newest
 * sample). Y axes auto-fit the visible data; caution / warning limits from
 * telemetry/monitor are drawn as shaded bands and lines; hovering any plot
 * shows all channels at that moment. Toolbar: pause (frozen snapshot), follow,
 * auto-Y, limits, time window, reset view, copy CSV. `sample_period_s` is the
 * cadence trends_sample() runs at. `open` is cleared when the user closes the
 * window. */
void trends_panel_draw(bool *open, const Trends *t, double sample_period_s);

#endif /* UI_TRENDS_PANEL_H */
