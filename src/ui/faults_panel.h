#ifndef UI_FAULTS_PANEL_H
#define UI_FAULTS_PANEL_H

#include "physics/cylinder.h"
#include "physics/engine_model.h"
#include "telemetry/event_log.h"

/* Dockable fault-injection window: pick a cylinder, drag its trims (injector
 * flow, compression, spark timing, intake leak, cooling) or apply a preset,
 * and the running model uses them on its next step. `cfg` is the live
 * per-cylinder config array (ModelSync::cyl_config). Edits are logged to `log`
 * as "FAULT" events. `open` is cleared when the user closes the window. */
void faults_panel_draw(bool *open, CylinderConfig cfg[ENGINE_MAX_CYLINDERS],
                       int num_cyl, EventLog *log, double sim_time_s);

#endif /* UI_FAULTS_PANEL_H */
