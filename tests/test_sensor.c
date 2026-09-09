#include "test_util.h"

#include "telemetry/sensor.h"
#include "model/state.h"

static ModelState make_truth(void) {
  ModelState t;
  t.engine.omega_rad_s = 220.0;
  t.engine.map_kpa = 78.0;
  t.thermal.cht_c = 180.0;
  t.thermal.egt_c = 720.0;
  t.thermal.oil_temp_c = 90.0;
  t.rpm = 2100.0;
  t.torque_nm = 60.0;
  return t;
}

static void test_config_defaults_are_sane(void) {
  SensorConfig c = sensor_config_default();
  CHECK(c.noise_rpm > 0.0);
  CHECK(c.noise_map_kpa > 0.0);
  CHECK(c.noise_cht_c > 0.0);
  CHECK(c.noise_egt_c > 0.0);
  CHECK(c.noise_oil_temp_c > 0.0);
  CHECK(c.dropout_probability >= 0.0);
  CHECK(c.dropout_probability < 1.0);
}

static void test_zero_noise_zero_dropout_is_exact_passthrough(void) {
  SensorConfig c = {0};
  Sensor s;
  sensor_init(&s, &c, 1u);
  ModelState truth = make_truth();
  for (int i = 0; i < 100; i++) {
    SensorReading r = sensor_read(&s, &truth);
    CHECK(r.value.rpm == truth.rpm);
    CHECK(r.value.map_kpa == truth.engine.map_kpa);
    CHECK(r.value.cht_c == truth.thermal.cht_c);
    CHECK(r.value.egt_c == truth.thermal.egt_c);
    CHECK(r.value.oil_temp_c == truth.thermal.oil_temp_c);
    CHECK(r.ok.rpm && r.ok.map_kpa && r.ok.cht_c && r.ok.egt_c &&
          r.ok.oil_temp_c);
  }
}

static void test_same_seed_is_reproducible(void) {
  SensorConfig c = sensor_config_default();
  Sensor a, b;
  sensor_init(&a, &c, 424242u);
  sensor_init(&b, &c, 424242u);
  ModelState truth = make_truth();
  for (int i = 0; i < 1000; i++) {
    SensorReading ra = sensor_read(&a, &truth);
    SensorReading rb = sensor_read(&b, &truth);
    CHECK(ra.value.rpm == rb.value.rpm);
    CHECK(ra.value.egt_c == rb.value.egt_c);
    CHECK(ra.ok.cht_c == rb.ok.cht_c);
  }
}

static void test_different_seed_diverges(void) {
  SensorConfig c = sensor_config_default();
  Sensor a, b;
  sensor_init(&a, &c, 1u);
  sensor_init(&b, &c, 2u);
  ModelState truth = make_truth();
  int differences = 0;
  for (int i = 0; i < 100; i++) {
    SensorReading ra = sensor_read(&a, &truth);
    SensorReading rb = sensor_read(&b, &truth);
    if (ra.value.rpm != rb.value.rpm) {
      differences++;
    }
  }
  CHECK(differences > 90);
}

static void test_seed_zero_is_replaced_and_still_runs(void) {
  SensorConfig c = sensor_config_default();
  Sensor s;
  sensor_init(&s, &c, 0u);
  CHECK(s.rng_state != 0u);
  ModelState truth = make_truth();
  SensorReading r = sensor_read(&s, &truth);
  CHECK(r.value.rpm > truth.rpm - 100.0);
  CHECK(r.value.rpm < truth.rpm + 100.0);
}

static void test_noise_is_unbiased_and_correctly_scaled(void) {
  SensorConfig c = sensor_config_default();
  c.dropout_probability = 0.0;
  Sensor s;
  sensor_init(&s, &c, 20240501u);
  ModelState truth = make_truth();

  const int N = 100000;
  double sum = 0.0, sumsq = 0.0;
  for (int i = 0; i < N; i++) {
    double err = sensor_read(&s, &truth).value.cht_c - truth.thermal.cht_c;
    sum += err;
    sumsq += err * err;
  }
  double mean = sum / N;
  double sd = sqrt(sumsq / N - mean * mean);
  CHECK_NEAR(mean, 0.0, 0.05 * c.noise_cht_c);
  CHECK_NEAR(sd, c.noise_cht_c, 0.10 * c.noise_cht_c);
}

static void test_dropout_rate_matches_config(void) {
  SensorConfig c = sensor_config_default();
  c.dropout_probability = 0.20;
  c.noise_rpm = 0.0;
  c.noise_map_kpa = 0.0;
  c.noise_cht_c = 0.0;
  c.noise_egt_c = 0.0;
  c.noise_oil_temp_c = 0.0;
  Sensor s;
  sensor_init(&s, &c, 777u);
  ModelState truth = make_truth();

  const int N = 100000;
  int dropped = 0;
  for (int i = 0; i < N; i++) {
    SensorReading r = sensor_read(&s, &truth);
    if (!r.ok.rpm) {
      dropped++;
      CHECK(r.value.rpm == 0.0); /* dropped channel is zeroed */
    }
  }
  CHECK_NEAR((double)dropped / N, 0.20, 0.02);
}

static const TestCase CASES[] = {
    {"sensor.config_defaults_are_sane", test_config_defaults_are_sane},
    {"sensor.zero_noise_zero_dropout_is_exact_passthrough",
     test_zero_noise_zero_dropout_is_exact_passthrough},
    {"sensor.same_seed_is_reproducible", test_same_seed_is_reproducible},
    {"sensor.different_seed_diverges", test_different_seed_diverges},
    {"sensor.seed_zero_is_replaced_and_still_runs",
     test_seed_zero_is_replaced_and_still_runs},
    {"sensor.noise_is_unbiased_and_correctly_scaled",
     test_noise_is_unbiased_and_correctly_scaled},
    {"sensor.dropout_rate_matches_config", test_dropout_rate_matches_config},
};

RUN_TESTS(CASES)
