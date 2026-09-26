#ifndef PHYSICS_ECU_DIAG_H
#define PHYSICS_ECU_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* The ECU's self-diagnosis of its two engine-speed inputs, its temperature
 * limits, and the fault codes it records */

typedef struct {
  double jump_rpm;     /* smallest reading step that counts as a jump */
  double mismatch_rpm; /* channels differing by more than this disagree */
  double mismatch_s;   /* ... for this long before a mismatch is declared */
  double frozen_s;     /* unchanged this long = frozen */
  double heal_s;       /* clean this long before a fault clears */

  double cht_high_c, cht_crit_c; /* head temperature limits */
  double egt_high_c, egt_crit_c; /* exhaust gas temperature limits */
  double oil_high_c, oil_crit_c; /* oil temperature limits */
  double temp_hold_s;            /* over a limit this long before it sets */
} EcuDiagConfig;

typedef enum {
  ECU_SRC_NONE = 0, /* no trusted speed */
  ECU_SRC_PRIMARY,
  ECU_SRC_SECONDARY
} EcuSpeedSource;

typedef enum {
  ECU_DTC_RPM1_JUMP = 0,
  ECU_DTC_RPM1_FROZEN,
  ECU_DTC_RPM1_LOST,
  ECU_DTC_RPM2_JUMP,
  ECU_DTC_RPM2_FROZEN,
  ECU_DTC_RPM2_LOST,
  ECU_DTC_MISMATCH,
  ECU_DTC_CHT_HIGH,
  ECU_DTC_CHT_CRIT,
  ECU_DTC_EGT_HIGH,
  ECU_DTC_EGT_CRIT,
  ECU_DTC_OIL_HIGH,
  ECU_DTC_OIL_CRIT,
  ECU_DTC_COUNT
} EcuDtcId;

#define ECU_TEMP_CHECKS 6 /* ECU_DTC_CHT_HIGH .. ECU_DTC_OIL_CRIT */

typedef struct {
  const char *code;        /* e.g. "E101" */
  const char *description; /* one line */
  int severity;            /* 0 caution, 1 warning */
} EcuDtcInfo;

/* Snapshot of the operating point when a code set. */
typedef struct {
  double rpm1, rpm2;
  double pilot_throttle;
  double throttle_cmd;
  EcuSpeedSource source;
  double cht_c, egt_c, oil_c;
} EcuFreezeFrame;

typedef struct {
  int active;  /* the fault is present now */
  int latched; /* it has occurred since the codes were last cleared */
  int count;   /* occurrences since the codes were last cleared */
  double first_s, last_s;
  EcuFreezeFrame freeze; /* as of the latest occurrence */
} EcuDtc;

typedef struct {
  double prev;
  int prev_valid;
  double frozen_ref, frozen_t;
  int jump, lost, frozen;
  double good_t; /* time since the channel last misbehaved */
} EcuSpeedChannel;

typedef struct {
  EcuSpeedChannel ch[2];
  double mismatch_t, agree_t;
  int mismatch;
  double temp_hold_t[ECU_TEMP_CHECKS];  /* time at or over each limit */
  double temp_clear_t[ECU_TEMP_CHECKS]; /* time clear under it, once set */
  int temp_active[ECU_TEMP_CHECKS];
  EcuDtc dtc[ECU_DTC_COUNT];
} EcuDiag;

typedef struct {
  double now_s, dt;
  double rpm[2];                       /* primary, secondary readings */
  double cht_c, egt_c, oil_c;          /* hottest head, hottest port, oil */
  int running;                         /* engine running with ignition on */
  double pilot_throttle, throttle_cmd; /* for the freeze frame */
} EcuDiagInput;

typedef struct {
  EcuSpeedSource source;
  int channel_fault[2]; /* the channel has a fault of its own */
  int mismatch;
} EcuDiagResult;

EcuDiagConfig ecu_diag_config_default(void);
void ecu_diag_init(EcuDiag *d);

/* Run the checks for one control step. */
void ecu_diag_step(EcuDiag *d, const EcuDiagConfig *cfg, const EcuDiagInput *in,
                   EcuDiagResult *out);

/* Maintenance clear: forgets the history (latched, counts, freeze frames). A
 * fault still present registers again as a new occurrence. */
void ecu_diag_clear_codes(EcuDiag *d);

/* Faults present now / codes that have ever set since the last clear. */
int ecu_diag_active_count(const EcuDiag *d);
int ecu_diag_latched_count(const EcuDiag *d);

const EcuDtcInfo *ecu_dtc_info(EcuDtcId id);

#ifdef __cplusplus
}
#endif

#endif /* PHYSICS_ECU_DIAG_H */
