#ifndef PHYSICS_ELECTRICAL_H
#define PHYSICS_ELECTRICAL_H

typedef struct {
  double bus_nominal_v; /* regulated bus voltage when the alternator carries the
                           load */
  double alt_rated_a;   /* alternator output at/above full-output rpm */
  double alt_cutin_rpm; /* below this the alternator makes ~nothing */
  double alt_full_output_rpm; /* at/above this it makes rated output */
  double alt_health;  /* 0..1: 1 healthy, 0 dead (belt slip, diode, winding) */
  double load_base_a; /* steady accessory draw (ECU, pumps, avionics) */
  double batt_capacity_ah;    /* battery capacity */
  double batt_open_v;         /* open-circuit terminal voltage at full charge */
  double batt_internal_r_ohm; /* internal resistance -> sag under discharge */
} ElecConfig;

typedef struct {
  double bus_v;         /* main bus voltage */
  double alt_current_a; /* alternator output current */
  double alt_field_a;   /* field / excitation current (proxy) */
  double batt_soc;      /* battery state of charge, 0..1 */
} ElecState;

/* Placeholder numbers for a small aircraft 14 V system. */
ElecConfig elec_config_default(void);

/* Full battery, engine not yet running. */
void elec_state_init(ElecState *state);

/* Refreshes the electrical state from the current crank speed. */
void elec_step(ElecState *state, const ElecConfig *config, double rpm,
               double dt);

#endif /* PHYSICS_ELECTRICAL_H */
