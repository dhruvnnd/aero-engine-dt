#ifndef UI_UI_HISTORY_H
#define UI_UI_HISTORY_H

/*
 * Fixed-capacity ring buffer of doubles for trend strips. No allocation: the
 * caller embeds a UiHistory (with a chosen CAP) wherever the sample series
 * needs to live -- typically one per plotted channel in the app state.
 *
 * Usage:
 *   UiHistory h; ui_history_init(&h, buf, CAP);
 *   ...each frame... ui_history_push(&h, sample);
 *   ...to draw... for (int i = 0; i < ui_history_count(&h); i++)
 *                   plot(ui_history_at(&h, i));   // i=0 oldest, last = newest
 */

typedef struct {
  double *buf; /* caller-owned storage, `cap` entries */
  int cap;     /* capacity */
  int count;   /* live samples, 0 .. cap */
  int head;    /* index of the oldest sample */
} UiHistory;

/* Bind `storage` (at least `cap` doubles) to the ring and clear it. */
void ui_history_init(UiHistory *h, double *storage, int cap);

/* Drop all samples without touching storage. */
void ui_history_clear(UiHistory *h);

/* Append one sample, evicting the oldest once full. */
void ui_history_push(UiHistory *h, double value);

/* Number of live samples. */
int ui_history_count(const UiHistory *h);

/* Sample by age index: 0 = oldest retained, count-1 = newest. Out-of-range
 * returns 0.0. */
double ui_history_at(const UiHistory *h, int i);

/* Most recent sample (0.0 if empty). */
double ui_history_last(const UiHistory *h);

/* Min over the live samples (0.0 if empty). */
double ui_history_min(const UiHistory *h);
/* Max over the live samples (0.0 if empty). */
double ui_history_max(const UiHistory *h);

#endif /* UI_UI_HISTORY_H */
