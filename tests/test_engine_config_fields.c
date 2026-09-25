#include <math.h>
#include <stdalign.h>
#include <stdio.h>
#include <string.h>

#include "physics/engine_config_fields.h"
#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"
#include "test_util.h"

#define TMP_PATH "test_engine_config_fields_tmp.cfg"

static double get(const EngineConfig *c, const ConfigField *f) {
  return *engine_config_field_cptr(c, f);
}

static void test_table_rows_are_well_formed(void) {
  CHECK(ENGINE_CONFIG_FIELD_COUNT > 0);
  CHECK(ENGINE_CONFIG_GROUP_COUNT > 0);

  const size_t size = sizeof(EngineConfig);
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    CHECK(f->key && f->key[0]);
    CHECK(f->label && f->label[0]);
    CHECK(f->unit != NULL);
    CHECK(f->doc && f->doc[0]);
    CHECK(f->fmt && strchr(f->fmt, '%') != NULL);
    CHECK(f->view_unit != NULL && f->view_scale > 0.0);
    CHECK(f->group >= 0 && f->group < ENGINE_CONFIG_GROUP_COUNT);
    CHECK(f->offset + sizeof(double) <= size);
    CHECK(f->offset % alignof(double) == 0);
    CHECK(f->slider_lo < f->slider_hi);

    for (int j = i + 1; j < ENGINE_CONFIG_FIELD_COUNT; j++) {
      const ConfigField *g = &ENGINE_CONFIG_FIELDS[j];
      CHECK(strcmp(f->key, g->key) != 0); /* unique keys */
      CHECK(f->offset != g->offset);      /* no two rows share a member */
    }
    CHECK(engine_config_find_field(f->key) == f);
  }
  CHECK(engine_config_find_field("no_such_key") == NULL);

  for (int g = 0; g < ENGINE_CONFIG_GROUP_COUNT; g++) {
    CHECK(ENGINE_CONFIG_GROUPS[g].name && ENGINE_CONFIG_GROUPS[g].name[0]);
  }
}

/* Every double in EngineConfig must have a row. If this fails, a double was
 * added to EngineConfig (or EngineGeometry) without a row in
 * ENGINE_CONFIG_FIELDS -- add one, or the spec file, the editor and the
 * validator will silently ignore it. (num_cylinders and firing_order are the
 * non-table members.) */
static void test_every_double_in_the_struct_has_a_row(void) {
  const size_t ints = (size_t)(1 + ENGINE_MAX_CYLINDERS) * sizeof(int);
  const size_t int_block = (ints + alignof(double) - 1) / alignof(double) *
                           alignof(double);
  const size_t expected =
      (size_t)ENGINE_CONFIG_FIELD_COUNT * sizeof(double) + int_block;
  CHECK(sizeof(EngineConfig) == expected);
}

static void test_defaults_satisfy_their_own_rows(void) {
  const EngineConfig d = engine_config_default();
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    const double v = get(&d, f);
    CHECK(engine_config_field_in_range(f, v));
    CHECK(v >= f->slider_lo && v <= f->slider_hi); /* reachable on the slider */
    if (f->typ_hi > f->typ_lo) {
      CHECK(engine_config_field_in_range(f, f->typ_lo));
      CHECK(engine_config_field_in_range(f, f->typ_hi));
    }
  }
  CHECK(engine_config_check(&d, NULL, 0) == 0);
}

static void test_range_bounds_are_enforced_per_field(void) {
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    CHECK(!engine_config_field_in_range(f, NAN));

    char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];

    if (isfinite(f->valid_min)) {
      const double bad =
          (f->flags & CFG_MIN_EXCL) ? f->valid_min : f->valid_min - 1.0;
      CHECK(!engine_config_field_in_range(f, bad));
      EngineConfig c = engine_config_default();
      *engine_config_field_ptr(&c, f) = bad;
      const int n = engine_config_check(&c, msgs, ENGINE_CONFIG_MAX_ISSUES);
      CHECK(n >= 1);
      int named = 0;
      for (int m = 0; m < n && m < ENGINE_CONFIG_MAX_ISSUES; m++) {
        named += strstr(msgs[m], f->key) != NULL;
      }
      CHECK(named >= 1);
    }
    if (isfinite(f->valid_max)) {
      const double bad =
          (f->flags & CFG_MAX_EXCL) ? f->valid_max : f->valid_max + 1.0;
      CHECK(!engine_config_field_in_range(f, bad));
      EngineConfig c = engine_config_default();
      *engine_config_field_ptr(&c, f) = bad;
      CHECK(engine_config_check(&c, msgs, ENGINE_CONFIG_MAX_ISSUES) >= 1);
    }
  }
}

static void test_range_messages_read_naturally(void) {
  char m[ENGINE_CONFIG_ISSUE_LEN];
  engine_config_field_range_message(engine_config_find_field("map_tau_s"), -2.0,
                                    m, sizeof m);
  CHECK(strcmp(m, "map_tau_s = -2: must be positive") == 0);

  engine_config_field_range_message(
      engine_config_find_field("compression_ratio"), 0.5, m, sizeof m);
  CHECK(strstr(m, "must be greater than 1") != NULL);

  engine_config_field_range_message(engine_config_find_field("evo_deg"), 800.0,
                                    m, sizeof m);
  CHECK(strstr(m, "must be in [0, 720)") != NULL);

  engine_config_field_range_message(
      engine_config_find_field("combustion_efficiency"), 1.5, m, sizeof m);
  CHECK(strstr(m, "must be in (0, 1]") != NULL);

  /* an unbounded row can still reject NaN */
  engine_config_field_range_message(
      engine_config_find_field("spark_base_btdc_deg"), NAN, m, sizeof m);
  CHECK(strstr(m, "not a valid number") != NULL);
}

static void test_cross_field_rules_remain(void) {
  EngineConfig c = engine_config_default();
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];

  c.geom.evo_deg = 600.0; /* >= ivc_deg */
  int n = engine_config_check(&c, msgs, ENGINE_CONFIG_MAX_ISSUES);
  CHECK(n == 1);
  CHECK(strstr(msgs[0], "evo_deg") != NULL && strstr(msgs[0], "ivc_deg") != NULL);

  c = engine_config_default();
  c.geom.conrod_len_m = c.geom.stroke_m / 2.0;
  n = engine_config_check(&c, msgs, ENGINE_CONFIG_MAX_ISSUES);
  CHECK(n == 1);
  CHECK(strstr(msgs[0], "conrod_len_m") != NULL);
}

static void test_every_field_round_trips_through_a_spec_file(void) {
  /* a distinct in-range value for every row */
  EngineConfig cfg = engine_config_default();
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    double v = 0.5 * (f->slider_lo + f->slider_hi) + 0.013 * (double)(i + 1);
    if (!engine_config_field_in_range(f, v)) {
      v = get(&cfg, f) * 1.01;
    }
    *engine_config_field_ptr(&cfg, f) = v;
  }
  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  EngineConfig loaded;
  const EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_OK);
  CHECK(r.unknown_keys == 0);
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    const double want = get(&cfg, f);
    CHECK_NEAR(get(&loaded, f), want, 1e-5 * fabs(want) + 1e-12);
  }
  CHECK(loaded.num_cylinders == cfg.num_cylinders);
  remove(TMP_PATH);
}

static void test_saved_file_names_and_documents_every_field(void) {
  const EngineConfig cfg = engine_config_default();
  CHECK(engine_spec_save(TMP_PATH, &cfg) == 0);

  FILE *fp = fopen(TMP_PATH, "r");
  CHECK(fp != NULL);
  if (!fp) {
    return;
  }
  static char text[32768];
  const size_t n = fread(text, 1, sizeof text - 1, fp);
  text[n] = '\0';
  fclose(fp);

  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    char line[128];
    snprintf(line, sizeof line, "\n%s = ", f->key);
    CHECK(strstr(text, line) != NULL);

    /* the first line of the doc appears as a comment */
    char first[160];
    const char *nl = strchr(f->doc, '\n');
    const int len = nl ? (int)(nl - f->doc) : (int)strlen(f->doc);
    snprintf(first, sizeof first, "# %.*s\n", len, f->doc);
    CHECK(strstr(text, first) != NULL);
  }
  for (int g = 0; g < ENGINE_CONFIG_GROUP_COUNT; g++) {
    char head[96];
    snprintf(head, sizeof head, "# --- %s ---", ENGINE_CONFIG_GROUPS[g].name);
    CHECK(strstr(text, head) != NULL);
  }
  remove(TMP_PATH);
}

static void test_each_key_loads_into_its_own_member_only(void) {
  const EngineConfig defaults = engine_config_default();
  for (int i = 0; i < ENGINE_CONFIG_FIELD_COUNT; i++) {
    const ConfigField *f = &ENGINE_CONFIG_FIELDS[i];
    FILE *fp = fopen(TMP_PATH, "w");
    CHECK(fp != NULL);
    if (!fp) {
      return;
    }
    fprintf(fp, "%s = 1.25\n", f->key);
    fclose(fp);

    EngineConfig loaded;
    const EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
    CHECK(r.status == ENGINE_SPEC_OK);
    CHECK_NEAR(get(&loaded, f), 1.25, 1e-12);
    for (int j = 0; j < ENGINE_CONFIG_FIELD_COUNT; j++) {
      if (j != i) {
        const ConfigField *o = &ENGINE_CONFIG_FIELDS[j];
        CHECK_NEAR(get(&loaded, o), get(&defaults, o), 1e-12);
      }
    }
  }
  remove(TMP_PATH);
}

static void test_a_bad_value_for_a_table_key_is_a_parse_error(void) {
  FILE *fp = fopen(TMP_PATH, "w");
  CHECK(fp != NULL);
  if (!fp) {
    return;
  }
  fprintf(fp, "# comment\nbore_m = eighty-four\n");
  fclose(fp);
  EngineConfig loaded;
  const EngineSpecResult r = engine_spec_load(TMP_PATH, &loaded);
  CHECK(r.status == ENGINE_SPEC_ERR_PARSE);
  CHECK(r.error_line == 2);
  remove(TMP_PATH);
}

static const TestCase cases[] = {
    {"table rows are well formed", test_table_rows_are_well_formed},
    {"every struct double has a row", test_every_double_in_the_struct_has_a_row},
    {"defaults satisfy their own rows", test_defaults_satisfy_their_own_rows},
    {"range bounds enforced per field", test_range_bounds_are_enforced_per_field},
    {"range messages read naturally", test_range_messages_read_naturally},
    {"cross-field rules remain", test_cross_field_rules_remain},
    {"every field round-trips", test_every_field_round_trips_through_a_spec_file},
    {"saved file names + documents every field",
     test_saved_file_names_and_documents_every_field},
    {"each key loads into its own member",
     test_each_key_loads_into_its_own_member_only},
    {"bad value for a table key is a parse error",
     test_a_bad_value_for_a_table_key_is_a_parse_error},
};

RUN_TESTS(cases)
