#include "test_util.h"

#include "physics/cylinder.h"
#include "physics/thermal.h"

/* The engine-wide oil node and the per-cylinder head / exhaust-port nodes, fed
 * synthetic heat inputs. (Against the real engine: test_cylinder,
 * test_model_sync.) */

#define AMB 15.0

static void run_oil(ThermalState *st, const ThermalConfig *cfg,
                    double friction_w, double heat_w, double cool_index,
                    double duration_s) {
  const double dt = 0.1;
  const int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    thermal_oil_step(st, cfg, friction_w, heat_w, cool_index, AMB, i * dt, dt);
  }
}

static CylinderThermalInput firing(double heat_w) {
  CylinderThermalInput in = {heat_w, 0.0, 0.0, 0.008, 480.0};
  return in;
}

static void run_cyl(CylinderState *st, const CylinderConfig *cc,
                    const CylinderThermalInput *in, double cool_index,
                    const ThermalConfig *tc, double duration_s) {
  const double dt = 0.1;
  const int n = (int)(duration_s / dt + 0.5);
  for (int i = 0; i < n; i++) {
    cylinder_step(st, cc, in, 80.0, AMB, tc, cool_index, i * dt, dt);
  }
}

static void test_init_puts_everything_at_ambient(void) {
  ThermalState st;
  thermal_init(&st, 15.0);
  CHECK_NEAR(st.cht_c, 15.0, 1e-9);
  CHECK_NEAR(st.egt_c, 15.0, 1e-9);
  CHECK_NEAR(st.oil_temp_c, 15.0, 1e-9);
}

static void test_config_time_constants_are_ordered(void) {
  ThermalConfig cfg = thermal_config_default();
  CHECK(cfg.egt_tau_s < cfg.cht_tau_s); /* exhaust gas responds fastest */
  CHECK(cfg.cht_tau_s < cfg.oil_tau_s); /* oil sump slowest */
}

static void test_cool_divisor_grows_with_airflow_and_is_bounded(void) {
  CHECK_NEAR(thermal_cool_divisor(1.0), 1.0, 1e-12);
  CHECK(thermal_cool_divisor(2.0) > thermal_cool_divisor(1.0));
  CHECK(thermal_cool_divisor(0.3) < thermal_cool_divisor(1.0));
  CHECK_NEAR(thermal_cool_divisor(0.0), thermal_cool_divisor(0.35), 1e-12);
  CHECK_NEAR(thermal_cool_divisor(50.0), thermal_cool_divisor(4.0), 1e-12);
}

/* No heat at all: the oil, head and exhaust port all sit at ambient. */
static void test_stopped_engine_holds_at_ambient(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  thermal_init(&st, 20.0);
  for (int i = 0; i < 3000; i++) {
    thermal_oil_step(&st, &cfg, 0.0, 0.0, 1.0, 20.0, i * 0.1, 0.1);
  }
  CHECK_NEAR(st.oil_temp_c, 20.0, 1e-6);

  CylinderState cs;
  cylinder_state_init(&cs, 20.0);
  CylinderConfig cc = cylinder_config_default();
  CylinderThermalInput idle = {0.0, 0.0, 0.0, 0.0, 20.0};
  for (int i = 0; i < 3000; i++) {
    cylinder_step(&cs, &cc, &idle, 100.0, 20.0, &cfg, 1.0, i * 0.1, 0.1);
  }
  CHECK_NEAR(cs.cht_c, 20.0, 1e-6);
  CHECK_NEAR(cs.egt_c, 20.0, 1e-6);
}

static void test_oil_settles_at_the_heat_balance(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  thermal_init(&st, AMB);
  run_oil(&st, &cfg, 5000.0, 30000.0, 1.0, 6000.0); /* >> oil tau */
  const double kw = cfg.oil_base_kw +
                    ((1.0 - cfg.friction_head_share) * 5000.0 +
                     cfg.oil_heat_share * 30000.0) /
                        1000.0;
  CHECK_NEAR(st.oil_temp_c, AMB + kw * cfg.oil_k_per_kw, 0.05);
}

static void test_oil_is_hotter_with_more_heat_and_less_cooling(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState lo, hi, weak_cooling;
  thermal_init(&lo, AMB);
  thermal_init(&hi, AMB);
  thermal_init(&weak_cooling, AMB);
  run_oil(&lo, &cfg, 2000.0, 10000.0, 1.0, 6000.0);
  run_oil(&hi, &cfg, 6000.0, 40000.0, 1.0, 6000.0);
  run_oil(&weak_cooling, &cfg, 2000.0, 10000.0, 0.4, 6000.0);
  CHECK(hi.oil_temp_c > lo.oil_temp_c + 5.0);
  CHECK(weak_cooling.oil_temp_c > lo.oil_temp_c + 3.0);
}

static void test_oil_response_speed_matches_its_time_constant(void) {
  ThermalConfig cfg = thermal_config_default();
  ThermalState st;
  thermal_init(&st, AMB);
  run_oil(&st, &cfg, 5000.0, 30000.0, 1.0, cfg.oil_tau_s);
  ThermalState settled;
  thermal_init(&settled, AMB);
  run_oil(&settled, &cfg, 5000.0, 30000.0, 1.0, 10.0 * cfg.oil_tau_s);
  /* one time constant: 1 - 1/e = 63% of the way */
  CHECK_NEAR((st.oil_temp_c - AMB) / (settled.oil_temp_c - AMB), 0.632, 0.01);
}

/* A head fed the same heat gets hotter as its cooling is reduced. */
static void test_head_follows_heat_and_cooling(void) {
  ThermalConfig tc = thermal_config_default();
  CylinderConfig nominal = cylinder_config_default();
  CylinderConfig weak = cylinder_config_default();
  weak.cooling_trim = 0.5;
  CylinderThermalInput in = firing(8000.0);

  CylinderState a, b, c;
  cylinder_state_init(&a, AMB);
  cylinder_state_init(&b, AMB);
  cylinder_state_init(&c, AMB);
  run_cyl(&a, &nominal, &in, 1.0, &tc, 1500.0);
  run_cyl(&b, &weak, &in, 1.0, &tc, 1500.0);
  CylinderThermalInput light = firing(1000.0);
  run_cyl(&c, &nominal, &light, 1.0, &tc, 1500.0);

  const double kw = tc.head_base_kw + tc.head_heat_share * 8.0;
  CHECK_NEAR(a.cht_c, AMB + kw * tc.cht_k_per_kw, 0.1);
  CHECK_NEAR(b.cht_c - AMB, 2.0 * (a.cht_c - AMB), 0.2);
  CHECK(c.cht_c < a.cht_c);
  CHECK(c.cht_c > AMB + 20.0); /* still warm: the hot gas heats it regardless */
}

/* A misfiring cylinder releases no heat, so its head and port stay cold. */
static void test_head_of_a_dead_cylinder_stays_cold(void) {
  ThermalConfig tc = thermal_config_default();
  CylinderConfig cc = cylinder_config_default();
  CylinderThermalInput dead = {0.0, 0.0, 0.0, 0.008, 12.0};
  CylinderState s;
  cylinder_state_init(&s, AMB);
  run_cyl(&s, &cc, &dead, 1.0, &tc, 1500.0);
  CHECK(s.cht_c < AMB + 1.0);
  CHECK(s.egt_c < AMB + 1.0);
}

/* The exhaust port follows the gas temperature at exhaust-valve opening, and
 * runs relatively cooler on a thin flow (idle) than a heavy one. */
static void test_exhaust_port_follows_blowdown_and_flow(void) {
  ThermalConfig tc = thermal_config_default();
  CylinderConfig cc = cylinder_config_default();
  CylinderThermalInput cool_gas = firing(8000.0);
  CylinderThermalInput hot_gas = firing(8000.0);
  hot_gas.blowdown_c = 650.0;
  CylinderThermalInput thin = firing(8000.0);
  thin.gas_flow_kg_s = 0.001;

  CylinderState a, b, c;
  cylinder_state_init(&a, AMB);
  cylinder_state_init(&b, AMB);
  cylinder_state_init(&c, AMB);
  run_cyl(&a, &cc, &cool_gas, 1.0, &tc, 100.0); /* >> egt tau */
  run_cyl(&b, &cc, &hot_gas, 1.0, &tc, 100.0);
  run_cyl(&c, &cc, &thin, 1.0, &tc, 100.0);
  CHECK(b.egt_c > a.egt_c + 100.0);
  CHECK(c.egt_c < a.egt_c - 50.0);
  CHECK_NEAR(a.egt_c,
             AMB + tc.egt_port_factor * (480.0 - AMB) -
                 tc.egt_port_loss_w / (0.008 * 1100.0),
             0.5);
}

/* A cylinder's exhaust port responds within seconds; its head takes minutes. */
static void test_port_is_faster_than_head(void) {
  ThermalConfig tc = thermal_config_default();
  CylinderConfig cc = cylinder_config_default();
  CylinderThermalInput in = firing(8000.0);
  CylinderState s;
  cylinder_state_init(&s, AMB);
  run_cyl(&s, &cc, &in, 1.0, &tc, 10.0);
  CylinderState settled;
  cylinder_state_init(&settled, AMB);
  run_cyl(&settled, &cc, &in, 1.0, &tc, 3000.0);
  const double egt_frac = (s.egt_c - AMB) / (settled.egt_c - AMB);
  const double cht_frac = (s.cht_c - AMB) / (settled.cht_c - AMB);
  CHECK(egt_frac > 0.8);
  CHECK(cht_frac < 0.15);
}

static void test_imep_is_reported_in_bar(void) {
  ThermalConfig tc = thermal_config_default();
  CylinderConfig cc = cylinder_config_default();
  CylinderThermalInput in = firing(8000.0);
  in.imep_kpa = 800.0;
  CylinderState s;
  cylinder_state_init(&s, AMB);
  cylinder_step(&s, &cc, &in, 80.0, AMB, &tc, 1.0, 0.0, 0.1);
  CHECK_NEAR(s.imep_bar, 8.0, 1e-9);
}

static const TestCase CASES[] = {
    {"thermal.init_puts_everything_at_ambient",
     test_init_puts_everything_at_ambient},
    {"thermal.config_time_constants_are_ordered",
     test_config_time_constants_are_ordered},
    {"thermal.cool_divisor_grows_with_airflow_and_is_bounded",
     test_cool_divisor_grows_with_airflow_and_is_bounded},
    {"thermal.stopped_engine_holds_at_ambient",
     test_stopped_engine_holds_at_ambient},
    {"thermal.oil_settles_at_the_heat_balance",
     test_oil_settles_at_the_heat_balance},
    {"thermal.oil_is_hotter_with_more_heat_and_less_cooling",
     test_oil_is_hotter_with_more_heat_and_less_cooling},
    {"thermal.oil_response_speed_matches_its_time_constant",
     test_oil_response_speed_matches_its_time_constant},
    {"thermal.head_follows_heat_and_cooling",
     test_head_follows_heat_and_cooling},
    {"thermal.head_of_a_dead_cylinder_stays_cold",
     test_head_of_a_dead_cylinder_stays_cold},
    {"thermal.exhaust_port_follows_blowdown_and_flow",
     test_exhaust_port_follows_blowdown_and_flow},
    {"thermal.port_is_faster_than_head", test_port_is_faster_than_head},
    {"thermal.imep_is_reported_in_bar", test_imep_is_reported_in_bar},
};

RUN_TESTS(CASES)
