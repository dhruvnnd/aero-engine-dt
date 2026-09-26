#ifndef UI_ECU_TRENDS_PANEL_H
#define UI_ECU_TRENDS_PANEL_H

#include "telemetry/ecu_trends.h"

void ecu_trends_panel_draw(bool *open, const EcuTrends *t,
                           const EngineConfig *cfg, double sample_period_s);

#endif /* UI_ECU_TRENDS_PANEL_H */
