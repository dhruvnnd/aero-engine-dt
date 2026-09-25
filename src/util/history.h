#ifndef UTIL_HISTORY_H
#define UTIL_HISTORY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed-capacity ring buffer of doubles for trend strips. No allocation: the
 * caller embeds a History (with a chosen CAP) wherever the sample series
 * needs to live -- typically one per plotted channel in the app state.
 *
 * Usage:
 *   History h; history_init(&h, buf, CAP);
 *   ...each frame... history_push(&h, sample);
 *   ...to draw... for (int i = 0; i < history_count(&h); i++)
 *                   plot(history_at(&h, i));   // i=0 oldest, last = newest
 */

typedef struct {
  double *buf; /* caller-owned storage, `cap` entries */
  int cap;     /* capacity */
  int count;   /* live samples, 0 .. cap */
  int head;    /* index of the oldest sample */
} History;

/* Bind `storage` (at least `cap` doubles) to the ring and clear it. */
void history_init(History *h, double *storage, int cap);

/* Drop all samples without touching storage. */
void history_clear(History *h);

/* Append one sample, evicting the oldest once full. */
void history_push(History *h, double value);

/* Number of live samples. */
int history_count(const History *h);

/* Sample by age index: 0 = oldest retained, count-1 = newest. Out-of-range
 * returns 0.0. */
double history_at(const History *h, int i);

/* Most recent sample (0.0 if empty). */
double history_last(const History *h);

/* Min over the live samples (0.0 if empty). */
double history_min(const History *h);
/* Max over the live samples (0.0 if empty). */
double history_max(const History *h);

#ifdef __cplusplus
}
#endif

#endif /* UTIL_HISTORY_H */
