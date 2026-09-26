#ifndef TELEMETRY_ENGINE_FAULTS_H
#define TELEMETRY_ENGINE_FAULTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "model/state.h"
#include "physics/cylinder.h"
#include "telemetry/monitor.h"

typedef struct {
  int faulty;        /* at least one trim is off nominal */
  double since_s;    /* when the cylinder became faulty */
  ChannelStatus now; /* what the monitors say about it now */
  int flagged;       /* they have flagged it since it became faulty */
  double first_flag_s;
  ChannelStatus worst; /* the worst they have said since it became faulty */
} CylFaultTrack;

typedef struct {
  CylFaultTrack cyl[ENGINE_MAX_CYLINDERS];
} EngineFaultTracker;

void engine_faults_init(EngineFaultTracker *t);

/* True if any trim of `c` is off nominal. */
int cylinder_has_fault(const CylinderConfig *c);

/* Writes a short description of the trims that are off nominal, e.g.
 * "compression 0.50, cooling 0.30". Empty if none. */
void cylinder_fault_text(const CylinderConfig *c, char *out, int out_size);

/* The worst status the limit monitors give one cylinder: CHT, EGT, lambda and
 * misfire rate against their limits. */
ChannelStatus cylinder_symptom_status(const CylinderState *c);

/* Update every cylinder's record from the current configs and state. Call at a
 * steady cadence with the sim time. Removing a fault forgets its record. */
void engine_faults_update(EngineFaultTracker *t, const CylinderConfig *cfg,
                          const ModelState *s, int num_cyl, double now_s);

/* One cylinder's numbers against the others' average. */
typedef struct {
  double d_cht_c; /* head temperature minus the others' mean */
  double d_egt_c;
  double lambda;
  double misfire_pct;
  double power_pct; /* mean torque over the newest cycle as % of the others';
                       < 0 when unknown (no trace, or the engine is not
                       making torque) */
} CylSymptoms;

/* `trace` may be NULL. With one cylinder there are no "others": deltas are 0
 * and power is unknown. */
CylSymptoms engine_faults_symptoms(const ModelState *s, int cyl, int num_cyl,
                                   const EngineTrace *trace);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_ENGINE_FAULTS_H */
