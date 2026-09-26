#ifndef UI_ECU_COMPARE_PANEL_H
#define UI_ECU_COMPARE_PANEL_H

#include "model/state.h"
#include "telemetry/ecu_compare.h"

bool ecu_compare_panel_draw(bool *open, const ModelState *with_ecu,
                            const ModelState *without_ecu,
                            const EcuCompareTrends *t, bool fitted,
                            double sample_period_s);

#endif /* UI_ECU_COMPARE_PANEL_H */
