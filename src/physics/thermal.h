#ifndef PHYSICS_THERMAL_H
#define PHYSICS_THERMAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Engine-wide temperatures. cht_c and egt_c are not integrated here: they are
 * the hottest head and the mean exhaust port over the cylinders' own thermal
 * nodes (physics/cylinder.h), refreshed by the caller. Only the oil is an
 * engine-wide node. */
typedef struct {
  double cht_c;      /* hottest cylinder head */
  double egt_c;      /* mean exhaust port gas temperature */
  double oil_temp_c; /* oil temperature */
} ThermalState;

typedef struct {
  double cht_tau_s;
  double egt_tau_s;
  double oil_tau_s;

  /* Head: chases ambient + (head_base_kw while the cylinder fires +
   * head_heat_share * combustion heat + friction_head_share * friction, kW) *
   * cht_k_per_kw, divided by the cooling airflow. The base term is the wall
   * heat the hot gas puts in however lightly the cylinder is loaded: the gas
   * is as hot at idle as at cruise, so the head does not cool off in
   * proportion to the load. */
  double head_base_kw;
  double head_heat_share; /* share of a cylinder's combustion heat the head
                             has to shed */
  double friction_head_share; /* share of the friction heat that lands in the
                                 heads (the rest goes to the oil) */
  double cht_k_per_kw;    /* K of head temperature per kW of head heat at
                             reference cooling */

  /* Exhaust port: chases ambient + egt_port_factor * (gas temperature at
   * exhaust-valve opening - ambient) - egt_port_loss_w / (gas flow * cp): the
   * port sheds a fixed heat to its walls, which cools a thin flow (idle) more
   * than a heavy one. The factor is above 1 because combustion_efficiency
   * lumps the model's wall losses, so its burnt gas is cooler than a real
   * engine's. */
  double egt_port_factor;
  double egt_port_loss_w;

  /* Oil: chases ambient + (oil_base_kw while the engine fires + (1 -
   * friction_head_share) * friction power + oil_heat_share * combustion heat,
   * kW) * oil_k_per_kw, divided by the cooling airflow. */
  double oil_base_kw; /* heat the sump collects from bearings and piston
                         cooling however lightly the engine is loaded */
  double oil_heat_share;
  double oil_k_per_kw;
} ThermalConfig;

ThermalConfig thermal_config_default(void);

/* Divisor applied to head and oil temperature rises for a cooling-air-flow
 * index. 1.0 at reference cooling; >1 when cooling is strong (fast/dense air),
 * <1 when it's weak (slow/thin air) */
double thermal_cool_divisor(double cool_index);

/* Sets all three temperatures to ambient (cold start). */
void thermal_init(ThermalState *state, double ambient_temp_c);

/* Advances the oil node by dt seconds. `friction_w` is the engine's friction
 * power, `combustion_heat_w` the total heat released in all cylinders, and
 * `cool_index` (from environment_cool_index) scales the oil cooling. */
void thermal_oil_step(ThermalState *state, const ThermalConfig *config,
                      double friction_w, double combustion_heat_w,
                      double cool_index, double ambient_temp_c, double t,
                      double dt);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_THERMAL_H */
