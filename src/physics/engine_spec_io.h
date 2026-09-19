#ifndef PHYSICS_ENGINE_SPEC_IO_H
#define PHYSICS_ENGINE_SPEC_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "physics/engine_model.h"

typedef enum {
  ENGINE_SPEC_OK = 0,
  ENGINE_SPEC_ERR_OPEN,  /* couldn't open the file for the requested mode */
  ENGINE_SPEC_ERR_PARSE, /* malformed line or unparsable value */
} EngineSpecStatus;

typedef struct {
  EngineSpecStatus status;
  int error_line;   /* 1-based line number of a parse error; 0 if none */
  int unknown_keys; /* count of unrecognized keys skipped (soft warning) */
} EngineSpecResult;

/* Loads an engine spec from `path` into *out
 * *out is set to engine_config_default() by default as fallback. */
EngineSpecResult engine_spec_load(const char *path, EngineConfig *out);

/* Writes *cfg to `path` in the same key=value format, with a header and one
 * comment per field documenting its unit and a typical range.
 * Returns 0 on success, -1 if `path` couldn't be
 * opened for writing (diagnostic printed to stderr). */
int engine_spec_save(const char *path, const EngineConfig *cfg);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_SPEC_IO_H */
