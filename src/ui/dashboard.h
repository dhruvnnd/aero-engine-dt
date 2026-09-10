#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#include <SDL3/SDL.h>

#include "model/state.h"
#include "ui/ui_history.h"
#include "ui/ui_theme.h"

#define DASHBOARD_TREND_CAP 300

typedef struct {
  UiTheme theme;

  double rpm_buf[DASHBOARD_TREND_CAP];
  double cht_buf[DASHBOARD_TREND_CAP];
  double egt_buf[DASHBOARD_TREND_CAP];
  double oilp_buf[DASHBOARD_TREND_CAP];
  double batt_buf[DASHBOARD_TREND_CAP];
  UiHistory rpm_hist;
  UiHistory cht_hist;
  UiHistory egt_hist;
  UiHistory oilp_hist;
  UiHistory batt_hist;
} Dashboard;

void dashboard_init(Dashboard *d);

/* Append the current readings to the trend histories. Call at a fixed cadence
 * (e.g. a few Hz), not once per render frame. */
void dashboard_sample(Dashboard *d, const ModelState *s);

/* Render the whole screen into a `w` x `h` logical area. `sensor_mode` is
 * purely for the header marker -- non-zero when `s` is the noisy instrument
 * feed, zero when it is the exact model state. */
void dashboard_draw(Dashboard *d, SDL_Renderer *r, float w, float h,
                    const ModelState *s, int num_cyl, double throttle,
                    double sim_time_s, float fps, int sensor_mode);

#endif /* UI_DASHBOARD_H */
