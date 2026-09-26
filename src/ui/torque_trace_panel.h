#ifndef UI_TORQUE_TRACE_PANEL_H
#define UI_TORQUE_TRACE_PANEL_H

#include "physics/engine_model.h"

/* Dockable ImPlot window showing the engine's sub-step trace over the newest
 * 720 deg crank cycle, swept against crank angle
 * Stacked, sharing the crank-angle axis:
 *   - one line per cylinder: net (gas + inertia) torque, gas torque, inertia
 *     torque or cylinder pressure, chosen in the toolbar; each cylinder's
 *     firing TDC is marked in its colour
 *   - total engine torque with the cycle mean
 *   - crank speed ripple
 * Toolbar: pause (frozen copy of the trace), auto-Y, what the cylinder plot
 * shows. `trace` may be NULL or empty. `open` is cleared when the user closes
 * the window. */
void torque_trace_panel_draw(bool *open, const EngineTrace *trace,
                             const EngineConfig *cfg);

#endif /* UI_TORQUE_TRACE_PANEL_H */
