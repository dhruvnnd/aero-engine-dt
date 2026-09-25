#ifndef UI_READOUT_PANELS_H
#define UI_READOUT_PANELS_H

#include "model/sim_clock.h"
#include "model/state.h"

/* Dockable ImGui readout windows. Each clears `*open` when the user closes it;
 * call between NewFrame/Render. Limits and status classification come from
 * telemetry/monitor. */

/* Sim time, throttle, frame rate, display feed, run state, sim clock controls
 * (pause / step / speed, edited through `clock`), key help. */
void sim_panel_draw(bool *open, const ModelState *s, double sim_time_s,
                    double throttle, float fps, bool sensor_mode,
                    SimClock *clock);

/* Every monitored channel: value, range bar, status token. */
void instruments_panel_draw(bool *open, const ModelState *s);

/* Flight condition: OAT, ambient pressure, density altitude, airspeed. */
void environment_panel_draw(bool *open, const ModelState *s);

/* Per-cylinder CHT / EGT / mixture / misfire, coloured by limit status. */
void cylinders_panel_draw(bool *open, const ModelState *s, int num_cyl);

#endif /* UI_READOUT_PANELS_H */
