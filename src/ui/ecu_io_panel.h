#ifndef UI_ECU_IO_PANEL_H
#define UI_ECU_IO_PANEL_H

#include "model/state.h"

struct EcuIoActions {
  bool set_rpm_fault; /* apply `kind` / `value` to the crank-speed input */
  EcuFaultKind kind;
  double value;
};

EcuIoActions ecu_io_panel_draw(bool *open, const ModelState *s,
                               const EcuSensorFault *rpm_fault);

#endif /* UI_ECU_IO_PANEL_H */
