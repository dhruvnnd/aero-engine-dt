#ifndef PHYSICS_LUBRICATION_H
#define PHYSICS_LUBRICATION_H

/* Oil pressure model.
 * The pump is crank-driven, so rail pressure follows RPM essentially instantly
 */

typedef struct {
  double relief_valve_kpa;   /* pressure the relief valve starts to bleed at */
  double relief_band_kpa;    /* how far above that pressure can still climb */
  double k_pump_kpa_per_rpm; /* pump gain at the reference oil viscosity */
  double visc_ref_temp_c;    /* oil temp where the viscosity factor is 1.0 */
  double visc_falloff_per_c; /* viscosity factor lost per degC above ref */
  double bearing_wear;       /* 0 = new; >0 widens clearances, drops pressure */
} LubeConfig;

typedef struct {
  double oil_press_kpa;
} LubeState;

/* Placeholder numbers for a small aero piston engine. */
LubeConfig lube_config_default(void);

/* Engine stopped: zero pressure. */
void lube_state_init(LubeState *state);

/* Refreshes oil pressure from crank speed and oil temperature. */
void lube_step(LubeState *state, const LubeConfig *config, double rpm,
               double oil_temp_c);

#endif /* PHYSICS_LUBRICATION_H */
