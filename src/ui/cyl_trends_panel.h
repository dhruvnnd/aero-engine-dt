#ifndef UI_CYL_TRENDS_PANEL_H
#define UI_CYL_TRENDS_PANEL_H

#include "telemetry/cyl_trends.h"

/* Dockable ImPlot window with one plot per cylinder metric (CHT, EGT, mixture,
 * misfire), one line per cylinder, stacked with a shared time axis (seconds
 * relative to now). Caution / warning limits are drawn where they exist;
 * hovering any plot shows every cylinder's readings at that moment. Toolbar:
 * pause (frozen snapshot), follow, auto-Y, limits, reset view.
 * `num_cyl` cylinders are shown; `sample_period_s` is the cadence
 * cyl_trends_sample() runs at. `open` is cleared when the user closes the
 * window. */
void cyl_trends_panel_draw(bool *open, const CylTrends *t, int num_cyl,
                           double sample_period_s);

#endif /* UI_CYL_TRENDS_PANEL_H */
