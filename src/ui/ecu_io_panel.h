#ifndef UI_ECU_IO_PANEL_H
#define UI_ECU_IO_PANEL_H

#include "model/state.h"

/* What the user asked for this frame; the caller performs it. */
struct EcuIoActions {
  bool set_fault; /* apply `kind` / `value` to speed sensor `channel` */
  int channel;    /* 0 = crank (primary), 1 = alternator (secondary) */
  EcuFaultKind kind;
  double value;
};

EcuIoActions ecu_io_panel_draw(bool *open, const ModelState *s,
                               const EcuSensorFault faults[2]);

#endif /* UI_ECU_IO_PANEL_H */
