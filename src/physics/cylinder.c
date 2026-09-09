#include "physics/cylinder.h"

CylinderConfig cylinder_config_default(void) {
  CylinderConfig cfg;
  cfg.injector_flow_trim = 1.0;
  cfg.compression_trim = 1.0;
  cfg.spark_offset_deg = 0.0;
  cfg.intake_leak_frac = 0.0;
  cfg.cooling_trim = 1.0;
  return cfg;
}

void cylinder_state_init(CylinderState *state, double ambient_temp_c) {
  state->cht_c = ambient_temp_c;
  state->egt_c = ambient_temp_c;
  state->lambda = 1.0; /* stoichiometric: neutral, and safe against /lambda */
  state->imep_bar = 0.0;
  state->ca50_deg = 0.0;
  state->fuel_pw_ms = 0.0;
  state->misfire_rate = 0.0;
}
