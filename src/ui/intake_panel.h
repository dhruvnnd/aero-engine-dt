#ifndef UI_INTAKE_PANEL_H
#define UI_INTAKE_PANEL_H

#include "telemetry/intake_trends.h"

void intake_panel_draw(bool *open, const IntakeTrends *t,
                       const EngineConfig *cfg, double sample_period_s);

#endif /* UI_INTAKE_PANEL_H */
