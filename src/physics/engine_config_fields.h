#ifndef PHYSICS_ENGINE_CONFIG_FIELDS_H
#define PHYSICS_ENGINE_CONFIG_FIELDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include "physics/engine_model.h"

/*
 * One row per numeric engine parameter, described once. The spec-file loader
 * and writer, the range validator, the Engine Spec viewer, the Spec Editor and
 * `twin_config --check` all walk this table, so adding a parameter is: the
 * struct member, its default in engine_config_default(), and one row here.
 *
 * Not in the table (they need custom handling): num_cylinders, firing_order,
 * and the cross-field rules (conrod vs stroke, evo before ivc).
 * tests/test_engine_config_fields.c fails if a double is added to
 * EngineConfig without a row.
 */

/* valid range flags: the bound itself is excluded */
#define CFG_MIN_EXCL 1
#define CFG_MAX_EXCL 2

typedef struct {
  const char *name;
  int advanced; /* model-tuning knobs: collapsed by default in the UI */
} ConfigGroup;

typedef struct {
  const char *key;   /* spec-file key, also the member name */
  const char *label; /* short human name */
  const char *unit;  /* unit of the stored value, "" if dimensionless */
  int group;         /* index into ENGINE_CONFIG_GROUPS */
  size_t offset;     /* byte offset of the double inside EngineConfig */

  double valid_min, valid_max; /* enforced by the validator; +-INFINITY = none */
  int flags;                   /* CFG_MIN_EXCL | CFG_MAX_EXCL */

  double typ_lo, typ_hi;       /* typical range (hint only); none if hi <= lo */
  double slider_lo, slider_hi; /* editor slider extent (typing may exceed it) */
  const char *fmt;             /* printf format for one value in the editor */

  double view_scale;     /* viewer shows value * view_scale ... */
  const char *view_unit; /* ... in this unit (e.g. m -> mm) */

  const char *doc; /* comment written above the key in spec files; '\n'
                    * separates lines */
} ConfigField;

extern const ConfigGroup ENGINE_CONFIG_GROUPS[];
extern const int ENGINE_CONFIG_GROUP_COUNT;
extern const ConfigField ENGINE_CONFIG_FIELDS[];
extern const int ENGINE_CONFIG_FIELD_COUNT;

/* The row for `key`, or NULL. */
const ConfigField *engine_config_find_field(const char *key);

double *engine_config_field_ptr(EngineConfig *cfg, const ConfigField *f);
const double *engine_config_field_cptr(const EngineConfig *cfg,
                                       const ConfigField *f);

/* True if `v` satisfies the row's valid range (NaN never does). */
int engine_config_field_in_range(const ConfigField *f, double v);

/* "bore_m = -1: must be positive" style message for an out-of-range value. */
void engine_config_field_range_message(const ConfigField *f, double v,
                                       char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_CONFIG_FIELDS_H */
