#ifndef PHYSICS_ENGINE_TRACE_H
#define PHYSICS_ENGINE_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Sub-step resolution debug trace of the engine model.
 * It is a fixed-size ring: the newest ENGINE_TRACE_CAPACITY samples are kept,
 * older ones are overwritten.
 * Purely an output -- attaching one does not change the simulation. */

#include "physics/engine_model.h"

#define ENGINE_TRACE_CAPACITY 8192

typedef struct {
  float theta_deg;   /* crank angle, [0,720) */
  float omega_rad_s; /* crank angular velocity */
  float torque_nm;   /* total crank torque (all cylinders + starter) */
  float cyl_gas_nm[ENGINE_MAX_CYLINDERS];     /* gas-pressure torque */
  float cyl_inertia_nm[ENGINE_MAX_CYLINDERS]; /* reciprocating-mass torque */
  float cyl_pressure_kpa[ENGINE_MAX_CYLINDERS];
} EngineTraceSample;

struct EngineTrace {
  EngineTraceSample samples[ENGINE_TRACE_CAPACITY];
  int head;  /* next write slot */
  int count; /* valid samples, <= ENGINE_TRACE_CAPACITY */
};

void engine_trace_clear(EngineTrace *t);
void engine_trace_push(EngineTrace *t, const EngineTraceSample *s);
int engine_trace_count(const EngineTrace *t);

/* i-th sample in chronological order (0 = oldest); NULL if out of range. */
const EngineTraceSample *engine_trace_at(const EngineTrace *t, int i);

/* How many of the newest samples span at most one 720 deg cycle of crank
 * travel (all of them if the trace covers less). The window is contiguous and
 * ends at the newest sample. */
int engine_trace_last_cycle_count(const EngineTrace *t);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ENGINE_TRACE_H */
