#include "test_util.h"

#include <string.h>

#include "model/channels.h"
#include "model/state.h"
#include "model/sync.h"
#include "physics/engine_model.h"

static void test_registry_is_wellformed(void) {
  int n = 0;
  const ModelChannel *ch = model_channels(&n);
  CHECK(n > 0);
  for (int i = 0; i < n; i++) {
    CHECK(ch[i].name != NULL && ch[i].name[0] != '\0');
    CHECK(ch[i].unit != NULL);
    CHECK(ch[i].get != NULL);
    CHECK(ch[i].precision >= 0);
    for (int j = i + 1; j < n; j++) {
      CHECK(strcmp(ch[i].name, ch[j].name) != 0); /* names are unique */
    }
  }
}

static void test_getters_match_state_fields(void) {
  ModelSync sync;
  model_sync_init(&sync);
  ModelState st;
  model_state_init(&st, &sync.engine_config, 15.0);

  EngineInput in;
  in.throttle = 0.8;
  in.load_torque_nm = 40.0;
  in.ambient_pressure_kpa = 101.325;
  EnvInput env = {0.0, 0.0, 0.0}; /* sea level, still air */
  for (int i = 0; i < 500; i++) {
    model_sync_step(&sync, &st, &in, &env, 0.01);
  }

  int n = 0;
  const ModelChannel *ch = model_channels(&n);
  for (int i = 0; i < n; i++) {
    double v = ch[i].get(&st, ch[i].index);
    if (strcmp(ch[i].name, "rpm") == 0) {
      CHECK_NEAR(v, st.rpm, 1e-9);
    } else if (strcmp(ch[i].name, "map_kpa") == 0) {
      CHECK_NEAR(v, st.engine.map_kpa, 1e-9);
    } else if (strcmp(ch[i].name, "torque_nm") == 0) {
      CHECK_NEAR(v, st.torque_nm, 1e-9);
    } else if (strcmp(ch[i].name, "cht_c") == 0) {
      CHECK_NEAR(v, st.thermal.cht_c, 1e-9);
    } else if (strcmp(ch[i].name, "egt_c") == 0) {
      CHECK_NEAR(v, st.thermal.egt_c, 1e-9);
    } else if (strcmp(ch[i].name, "oil_c") == 0) {
      CHECK_NEAR(v, st.thermal.oil_temp_c, 1e-9);
    }
  }
}

static const TestCase CASES[] = {
    {"channels.registry_is_wellformed", test_registry_is_wellformed},
    {"channels.getters_match_state_fields", test_getters_match_state_fields},
};

RUN_TESTS(CASES)
