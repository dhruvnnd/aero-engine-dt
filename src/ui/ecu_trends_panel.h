#ifndef UI_ECU_TRENDS_PANEL_H
#define UI_ECU_TRENDS_PANEL_H

#include "telemetry/ecu_trends.h"

/* `fitted` = the engine has an ECU; without one the panel just says so. */
void ecu_trends_panel_draw(bool *open, const EcuTrends *t,
                           const EngineConfig *cfg, bool fitted,
                           double sample_period_s);

#endif /* UI_ECU_TRENDS_PANEL_H */
