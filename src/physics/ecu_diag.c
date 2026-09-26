#include "physics/ecu_diag.h"

#include <math.h>
#include <string.h>

#define RATE_LIMIT_RPM_S 3000.0

/* Below this a running engine's speed reads as lost. */
#define LOST_BELOW_RPM 50.0

/* A reading changing by less than this counts as "not changed". */
#define FROZEN_BAND_RPM 0.05

static const EcuDtcInfo DTC_INFO[ECU_DTC_COUNT] = {
    {"E101", "Crank speed: implausible jump", 1},
    {"E102", "Crank speed: signal frozen", 1},
    {"E103", "Crank speed: signal lost", 1},
    {"E111", "Alternator speed: implausible jump", 0},
    {"E112", "Alternator speed: signal frozen", 0},
    {"E113", "Alternator speed: signal lost", 0},
    {"E120", "Speed sensors disagree, source unknown", 1},
};

const EcuDtcInfo *ecu_dtc_info(EcuDtcId id) {
  return (int)id >= 0 && (int)id < (int)ECU_DTC_COUNT ? &DTC_INFO[id] : NULL;
}

EcuDiagConfig ecu_diag_config_default(void) {
  EcuDiagConfig c;
  c.jump_rpm = 50.0;
  c.mismatch_rpm = 60.0;
  c.mismatch_s = 1.0;
  c.frozen_s = 2.0;
  c.heal_s = 5.0;
  return c;
}

void ecu_diag_init(EcuDiag *d) { memset(d, 0, sizeof *d); }

static int channel_faulty(const EcuSpeedChannel *c) {
  return c->jump || c->lost || c->frozen;
}

static void suspend_channel(EcuSpeedChannel *c) {
  c->prev_valid = 0;
  c->frozen_t = 0.0;
}

/* One channel's own checks; returns 1 if it misbehaved this step. */
static int check_channel(EcuSpeedChannel *c, const EcuDiagConfig *cfg, double x,
                         double dt) {
  int violation = 0;
  if (c->prev_valid) {
    const double delta = fabs(x - c->prev);
    if (delta > cfg->jump_rpm && delta > RATE_LIMIT_RPM_S * dt) {
      c->jump = 1;
      violation = 1;
    }
  }
  c->prev = x;
  c->prev_valid = 1;

  if (x < LOST_BELOW_RPM) {
    c->lost = 1;
    violation = 1;
  }

  if (fabs(x - c->frozen_ref) > FROZEN_BAND_RPM) {
    c->frozen_ref = x;
    c->frozen_t = 0.0;
  } else {
    c->frozen_t += dt;
    if (c->frozen_t >= cfg->frozen_s) {
      c->frozen = 1;
      violation = 1;
    }
  }

  c->good_t = violation ? 0.0 : c->good_t + dt;
  return violation;
}

static void record(EcuDiag *d, EcuDtcId id, int present, const EcuDiagInput *in,
                   EcuSpeedSource source) {
  EcuDtc *t = &d->dtc[id];
  if (present && !t->active) {
    t->count++;
    t->latched = 1;
    if (t->count == 1) {
      t->first_s = in->now_s;
    }
    t->last_s = in->now_s;
    t->freeze.rpm1 = in->rpm[0];
    t->freeze.rpm2 = in->rpm[1];
    t->freeze.pilot_throttle = in->pilot_throttle;
    t->freeze.throttle_cmd = in->throttle_cmd;
    t->freeze.source = source;
  }
  t->active = present;
}

void ecu_diag_step(EcuDiag *d, const EcuDiagConfig *cfg, const EcuDiagInput *in,
                   EcuDiagResult *out) {
  if (!in->running) {
    /* nothing is checked while the engine is not running; faults are held */
    suspend_channel(&d->ch[0]);
    suspend_channel(&d->ch[1]);
    d->mismatch_t = 0.0;
    d->agree_t = 0.0;
  } else {
    for (int c = 0; c < 2; c++) {
      check_channel(&d->ch[c], cfg, in->rpm[c], in->dt);
    }

    /* a faulty channel clears once it has been clean, and agreeing with the
     * other channel, for the heal time */
    const int agree = fabs(in->rpm[0] - in->rpm[1]) <= 0.5 * cfg->mismatch_rpm;
    for (int c = 0; c < 2; c++) {
      EcuSpeedChannel *ch = &d->ch[c];
      if (channel_faulty(ch) && ch->good_t >= cfg->heal_s && agree) {
        ch->jump = ch->lost = ch->frozen = 0;
        ch->good_t = 0.0;
      }
    }

    /* across channels: only meaningful if neither has a fault of its own */
    if (channel_faulty(&d->ch[0]) || channel_faulty(&d->ch[1])) {
      d->mismatch = 0; /* the culprit is known */
      d->mismatch_t = 0.0;
      d->agree_t = 0.0;
    } else {
      const double diff = fabs(in->rpm[0] - in->rpm[1]);
      if (diff > cfg->mismatch_rpm) {
        d->mismatch_t += in->dt;
        d->agree_t = 0.0;
        if (d->mismatch_t >= cfg->mismatch_s) {
          d->mismatch = 1;
        }
      } else {
        d->mismatch_t = 0.0;
        if (d->mismatch) {
          d->agree_t =
              diff <= 0.5 * cfg->mismatch_rpm ? d->agree_t + in->dt : 0.0;
          if (d->agree_t >= cfg->heal_s) {
            d->mismatch = 0;
            d->agree_t = 0.0;
          }
        }
      }
    }
  }

  out->channel_fault[0] = channel_faulty(&d->ch[0]);
  out->channel_fault[1] = channel_faulty(&d->ch[1]);
  out->mismatch = d->mismatch;
  if (d->mismatch) {
    out->source = ECU_SRC_NONE;
  } else if (!out->channel_fault[0]) {
    out->source = ECU_SRC_PRIMARY;
  } else if (!out->channel_fault[1]) {
    out->source = ECU_SRC_SECONDARY;
  } else {
    out->source = ECU_SRC_NONE;
  }

  record(d, ECU_DTC_RPM1_JUMP, d->ch[0].jump, in, out->source);
  record(d, ECU_DTC_RPM1_FROZEN, d->ch[0].frozen, in, out->source);
  record(d, ECU_DTC_RPM1_LOST, d->ch[0].lost, in, out->source);
  record(d, ECU_DTC_RPM2_JUMP, d->ch[1].jump, in, out->source);
  record(d, ECU_DTC_RPM2_FROZEN, d->ch[1].frozen, in, out->source);
  record(d, ECU_DTC_RPM2_LOST, d->ch[1].lost, in, out->source);
  record(d, ECU_DTC_MISMATCH, d->mismatch, in, out->source);
}

void ecu_diag_clear_codes(EcuDiag *d) {
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    EcuDtc *t = &d->dtc[i];
    t->active = 0; /* a fault still present registers again next step */
    t->latched = 0;
    t->count = 0;
    t->first_s = t->last_s = 0.0;
    memset(&t->freeze, 0, sizeof t->freeze);
  }
}

int ecu_diag_active_count(const EcuDiag *d) {
  int n = 0;
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    n += d->dtc[i].active != 0;
  }
  return n;
}

int ecu_diag_latched_count(const EcuDiag *d) {
  int n = 0;
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    n += d->dtc[i].latched != 0;
  }
  return n;
}
