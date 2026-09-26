#ifndef UI_ECU_PANEL_H
#define UI_ECU_PANEL_H

#include "model/state.h"

/* What the user asked for this frame; the caller performs it. */
struct EcuActions {
  bool toggle_idle_governor;
};

EcuActions ecu_panel_draw(bool *open, const ModelState *s,
                          const EngineConfig *cfg);

#endif /* UI_ECU_PANEL_H */
