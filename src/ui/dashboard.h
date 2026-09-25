#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <SDL3/SDL.h>

#include "model/state.h"
#include "telemetry/trends.h"
#include "ui/ui_theme.h"

typedef struct {
  UiTheme theme;
} Dashboard;

void dashboard_init(Dashboard *d);

/* Render the whole screen into a `w` x `h` logical area. `sensor_mode` is
 * purely for the header marker -- non-zero when `s` is the noisy instrument
 * feed, zero when it is the exact model state. `trends` feeds the sparklines. */
void dashboard_draw(Dashboard *d, SDL_Renderer *r, float w, float h,
                    const ModelState *s, const Trends *trends, int num_cyl,
                    double throttle, double sim_time_s, float fps,
                    int sensor_mode);

#ifdef __cplusplus
}
#endif

#endif /* UI_DASHBOARD_H */
