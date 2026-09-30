#ifndef UI_PV_DIAGRAM_PANEL_H
#define UI_PV_DIAGRAM_PANEL_H

#include "physics/cylinder.h"
#include "physics/engine_model.h"

void pv_diagram_panel_draw(bool *open, const EngineTrace *trace,
                           const EngineConfig *cfg,
                           const CylinderConfig *cyl_cfg);

#endif /* UI_PV_DIAGRAM_PANEL_H */
