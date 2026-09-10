#ifndef UI_UI_WIDGETS_H
#define UI_UI_WIDGETS_H

#include <SDL3/SDL.h>

#include "ui/ui_history.h"
#include "ui/ui_layout.h"
#include "ui/ui_theme.h"

typedef enum {
  UI_OK = 0, /* within limits */
  UI_WARN,   /* caution band */
  UI_ALERT,  /* warning band */
  UI_STALE   /* no / expired data */
} UiStatus;

/* Scale endpoints plus optional caution/warning thresholds on either side.
 * A `*_lo` limit trips when value <= it; a `*_hi` limit when value >= it.
 * Each threshold is ignored unless its matching has_* flag is set, so a
 * one-sided range simply leaves the unused pair zeroed. */
typedef struct {
  double lo, hi;               /* scale endpoints (bar length / dial sweep) */
  double warn_lo, warn_hi;     /* caution thresholds (amber) */
  double alert_lo, alert_hi;   /* warning thresholds (red) */
  int has_warn_lo, has_warn_hi;
  int has_alert_lo, has_alert_hi;
} UiRange;

/* Classify `value` against a range's caution/warning limits (worst side wins). */
UiStatus ui_status_for(double value, UiRange range);

/* Theme colour for a status (UI_OK -> normal text, not the `ok` green, so a
 * screen full of nominal readings stays monochrome). */
SDL_Color ui_status_color(const UiTheme *th, UiStatus s);

/* Fixed-width bracketed token: "[ OK ]" "[WARN]" "[ALRM]" "[----]". */
const char *ui_status_token(UiStatus s);

/* Titled box. Draws the outline, the title (upper-cased by the caller if
 * desired) and a rule beneath it. Returns the padded inner rect for content;
 * pass "" or NULL for an untitled box. */
UiRect ui_panel(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                const char *title);

/* One line: "NAME .......... 1234.5 unit", value right-aligned and coloured by
 *  `status`. Height ~ th->row_h. */
void ui_reading_row(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                    const char *name, double value, const char *unit,
                    int precision, UiStatus status);

/* Boxed key figure: small label on top, large-reading value + unit below.
 * (Font is fixed size, so "large" just means its own row, brightly drawn.) */
void ui_stat_tile(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                  const char *label, double value, const char *unit,
                  int precision);

/* Label + value row, then a horizontal track from range.lo to range.hi with a
 * filled bar to `value` and tick marks at each caution/warning limit that is
 * set (either side). Fill colour follows ui_status_for(value, range). */
void ui_bar_gauge(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                  const char *label, double value, const char *unit,
                  int precision, UiRange range);

/* Round "steam gauge" dial: a 270-degree scale arc (gap at the bottom) from
 * range.lo at lower-left, clockwise to range.hi at lower-right, with major
 * tick marks, amber/red band arcs at each caution/warning limit, a pointer at
 *  `value`, and a digital readout under the hub. `label` sits above the dial.
 * The dial is centred in whatever the box allows after those two text rows. */
void ui_dial_gauge(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                   const char *label, double value, const char *unit,
                   int precision, UiRange range);

/* Trend strip fed by a history ring: caption + latest value on the first row,
 * the plotted line filling the rest, min/max labels at the right edge. The
 * vertical scale auto-fits the samples unless range has positive span, in
 * which case [range.lo, range.hi] is used. */
void ui_sparkline(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                  const char *label, const char *unit, int precision,
                  const UiHistory *hist, UiRange range);

/* Grouped vertical bars for comparing a small set (e.g. per-cylinder CHT).
 *  `vals[i]` is drawn against [range.lo, range.hi]; each bar is coloured by
 * ui_status_for(vals[i], range). `tags` (may be NULL) labels the bars under
 * the baseline; NULL falls back to "1".."n". */
void ui_bar_series(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                   const char *label, const double *vals,
                   const char *const *tags, int n, const char *unit,
                   int precision, UiRange range);

#endif /* UI_UI_WIDGETS_H */
