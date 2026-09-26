#ifndef UI_ECU_FAULTS_PANEL_H
#define UI_ECU_FAULTS_PANEL_H

#include "model/state.h"

struct EcuFaultsActions {
  bool toggle_diagnostics;
  bool clear_codes;
};

EcuFaultsActions ecu_faults_panel_draw(bool *open, const ModelState *s);

#endif /* UI_ECU_FAULTS_PANEL_H */
