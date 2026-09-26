#ifndef UI_ENGINE_FAULTS_PANEL_H
#define UI_ENGINE_FAULTS_PANEL_H

#include "model/state.h"
#include "physics/cylinder.h"
#include "telemetry/engine_faults.h"

void engine_faults_panel_draw(bool *open, const CylinderConfig *cfg,
                              int num_cyl, const ModelState *s,
                              const EngineTrace *trace,
                              const EngineFaultTracker *tracker, double now_s);

#endif /* UI_ENGINE_FAULTS_PANEL_H */
