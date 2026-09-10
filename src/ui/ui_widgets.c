#include "ui/ui_widgets.h"

#include "ui/ui_draw.h"

static const float kGlyph = (float)SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;

#define UI_SPARK_MAX 512

UiStatus ui_status_for(double value, UiRange range) {
  if ((range.has_alert_lo && value <= range.alert_lo) ||
      (range.has_alert_hi && value >= range.alert_hi)) {
    return UI_ALERT;
  }
  if ((range.has_warn_lo && value <= range.warn_lo) ||
      (range.has_warn_hi && value >= range.warn_hi)) {
    return UI_WARN;
  }
  return UI_OK;
}

SDL_Color ui_status_color(const UiTheme *th, UiStatus s) {
  switch (s) {
  case UI_WARN:
    return th->warn;
  case UI_ALERT:
    return th->alert;
  case UI_STALE:
    return th->text_dim;
  case UI_OK:
  default:
    return th->text;
  }
}

/* Vertical tick on a bar-gauge track at scale position `at`. */
static void ui_gauge_tick(SDL_Renderer *r, UiRect track, UiRange range,
                          double at, SDL_Color c) {
  float tx =
      track.x + (float)ui_map(at, range.lo, range.hi, 0.0, (double)track.w);
  ui_vline(r, tx, track.y - 2.0f, track.h + 4.0f, c);
}

const char *ui_status_token(UiStatus s) {
  switch (s) {
  case UI_WARN:
    return "[WARN]";
  case UI_ALERT:
    return "[ALRM]";
  case UI_STALE:
    return "[----]";
  case UI_OK:
  default:
    return "[ OK ]";
  }
}

UiRect ui_panel(SDL_Renderer *r, UiRect bounds, const UiTheme *th,
                const char *title) {
  ui_box(r, bounds, th->frame);
  UiRect inner = ui_rect_inset(bounds, th->pad, th->pad);
  if (!ui_rect_valid(inner)) {
    return inner;
  }
  if (title != NULL && title[0] != '\0') {
    UiRect rest;
    UiRect head = ui_split_top(inner, kGlyph, 4.0f, &rest);
    ui_text(r, head.x, head.y, th->text_dim, "%s", title);
    ui_hline(r, inner.x, head.y + kGlyph + 2.0f, inner.w, th->frame);
    return rest;
  }
  return inner;
}

void ui_reading_row(SDL_Renderer *r, UiRect b, const UiTheme *th,
                    const char *name, double value, const char *unit,
                    int precision, UiStatus status) {
  if (!ui_rect_valid(b)) {
    return;
  }
  float y = b.y + (b.h - kGlyph) * 0.5f;
  char val[64];
  SDL_snprintf(val, sizeof(val), "%.*f %s", precision, value,
               unit != NULL ? unit : "");

  ui_text(r, b.x, y, th->text, "%s", name);
  float name_end = b.x + ui_text_w(name) + 4.0f;
  float val_start = b.x + b.w - ui_text_w(val) - 4.0f;
  ui_dot_leader(r, name_end, val_start, y + kGlyph - 1.0f, th->grid);
  ui_text_right(r, b.x + b.w, y, ui_status_color(th, status), "%s", val);
}

void ui_stat_tile(SDL_Renderer *r, UiRect b, const UiTheme *th,
                  const char *label, double value, const char *unit,
                  int precision) {
  ui_box(r, b, th->frame);
  UiRect in = ui_rect_inset(b, th->pad, th->pad);
  if (!ui_rect_valid(in)) {
    return;
  }
  ui_text(r, in.x, in.y, th->text_dim, "%s", label);
  float vy = in.y + in.h - kGlyph;
  ui_text(r, in.x, vy, th->text_bright, "%.*f", precision, value);
  if (unit != NULL && unit[0] != '\0') {
    ui_text_right(r, in.x + in.w, vy, th->text_dim, "%s", unit);
  }
}

void ui_bar_gauge(SDL_Renderer *r, UiRect b, const UiTheme *th,
                  const char *label, double value, const char *unit,
                  int precision, UiRange range) {
  if (!ui_rect_valid(b)) {
    return;
  }
  UiStatus st = ui_status_for(value, range);

  UiRect rest;
  UiRect top = ui_split_top(b, kGlyph, 3.0f, &rest);
  ui_text(r, top.x, top.y, th->text, "%s", label);
  ui_text_right(r, top.x + top.w, top.y, ui_status_color(th, st), "%.*f %s",
                precision, value, unit != NULL ? unit : "");

  UiRect track = rest;
  if (track.h > 8.0f) {
    track.h = 8.0f;
  }
  if (!ui_rect_valid(track)) {
    return;
  }
  ui_box(r, track, th->frame);

  UiRect fill = ui_rect_inset(track, 1.0f, 1.0f);
  fill.w = (float)(fill.w * ui_map(value, range.lo, range.hi, 0.0, 1.0));
  if (fill.w > 0.0f) {
    ui_fill(r, fill, ui_status_color(th, st));
  }

  if (range.has_warn_lo) {
    ui_gauge_tick(r, track, range, range.warn_lo, th->warn);
  }
  if (range.has_warn_hi) {
    ui_gauge_tick(r, track, range, range.warn_hi, th->warn);
  }
  if (range.has_alert_lo) {
    ui_gauge_tick(r, track, range, range.alert_lo, th->alert);
  }
  if (range.has_alert_hi) {
    ui_gauge_tick(r, track, range, range.alert_hi, th->alert);
  }
}

void ui_sparkline(SDL_Renderer *r, UiRect b, const UiTheme *th,
                  const char *label, const char *unit, int precision,
                  const UiHistory *hist, UiRange range) {
  if (!ui_rect_valid(b)) {
    return;
  }
  UiRect plot;
  UiRect top = ui_split_top(b, kGlyph, 3.0f, &plot);

  int n = ui_history_count(hist);
  ui_text(r, top.x, top.y, th->text_dim, "%s", label);
  ui_text_right(r, top.x + top.w, top.y, th->text, "%.*f %s", precision,
                ui_history_last(hist), unit != NULL ? unit : "");

  ui_box(r, plot, th->frame);
  UiRect pin = ui_rect_inset(plot, 2.0f, 2.0f);
  if (n < 2 || !ui_rect_valid(pin)) {
    return;
  }

  double vmin;
  double vmax;
  if (range.hi > range.lo) {
    vmin = range.lo;
    vmax = range.hi;
  } else {
    vmin = ui_history_min(hist);
    vmax = ui_history_max(hist);
    if (vmax - vmin < 1e-9) {
      vmax = vmin + 1.0;
    }
  }

  float label_w = 7.0f * kGlyph;
  float plot_w = pin.w - label_w - 4.0f;
  if (plot_w < 8.0f) {
    plot_w = pin.w; /* too narrow for edge labels; use full width */
    label_w = 0.0f;
  }

  int m = n > UI_SPARK_MAX ? UI_SPARK_MAX : n;
  int start = n - m;
  SDL_FPoint pts[UI_SPARK_MAX];
  for (int i = 0; i < m; i++) {
    double v = ui_history_at(hist, start + i);
    float x = pin.x + (m == 1 ? 0.0f : (float)i / (float)(m - 1) * plot_w);
    float y = (float)ui_map(v, vmin, vmax, (double)(pin.y + pin.h - 1.0f),
                            (double)pin.y);
    pts[i].x = x;
    pts[i].y = y;
  }
  ui_polyline(r, pts, m, th->text);

  if (label_w > 0.0f) {
    ui_text_right(r, plot.x + plot.w - 2.0f, pin.y, th->text_dim, "%.*f",
                  precision, vmax);
    ui_text_right(r, plot.x + plot.w - 2.0f, pin.y + pin.h - kGlyph,
                  th->text_dim, "%.*f", precision, vmin);
  }
}

void ui_bar_series(SDL_Renderer *r, UiRect b, const UiTheme *th,
                   const char *label, const double *vals,
                   const char *const *tags, int n, const char *unit,
                   int precision, UiRange range) {
  if (!ui_rect_valid(b) || n < 1 || vals == NULL) {
    return;
  }
  UiRect body;
  UiRect top = ui_split_top(b, kGlyph, 3.0f, &body);
  ui_text(r, top.x, top.y, th->text_dim, "%s", label);
  if (unit != NULL && unit[0] != '\0') {
    ui_text_right(r, top.x + top.w, top.y, th->text_dim, "%s", unit);
  }

  UiRect mid;
  UiRect valrow = ui_split_top(body, kGlyph, 2.0f, &mid);
  UiRect bars;
  UiRect tagrow = ui_split_bottom(mid, kGlyph, 2.0f, &bars);
  if (!ui_rect_valid(bars)) {
    return;
  }
  ui_box(r, bars, th->frame);
  UiRect in = ui_rect_inset(bars, 3.0f, 3.0f);
  if (!ui_rect_valid(in) || n <= 0) {
    return;
  }

  float slot = in.w / (float)n;
  float bw = slot * 0.55f;
  for (int i = 0; i < n; i++) {
    UiStatus st = ui_status_for(vals[i], range);
    float bh = (float)(in.h * ui_map(vals[i], range.lo, range.hi, 0.0, 1.0));
    UiRect bar = {in.x + slot * (float)i + (slot - bw) * 0.5f, in.y + in.h - bh,
                  bw, bh};
    if (bh > 0.0f) {
      ui_fill(r, bar, ui_status_color(th, st));
    }

    float cx = in.x + slot * ((float)i + 0.5f);
    char tg[16];
    if (tags != NULL && tags[i] != NULL) {
      SDL_snprintf(tg, sizeof(tg), "%s", tags[i]);
    } else {
      SDL_snprintf(tg, sizeof(tg), "%d", i + 1);
    }
    ui_text_center(r, cx, tagrow.y, th->text_dim, "%s", tg);
    ui_text_center(r, cx, valrow.y, ui_status_color(th, st), "%.*f", precision,
                   vals[i]);
  }
}

#define UI_DIAL_START 135.0f /* lower-left, degrees (screen convention) */
#define UI_DIAL_SWEEP 270.0f /* clockwise to lower-right */
#define UI_DIAL_D2R 0.017453292519943295f

static float ui_dial_angle(double value, UiRange range) {
  double t = ui_map(value, range.lo, range.hi, 0.0, 1.0);
  return UI_DIAL_START + UI_DIAL_SWEEP * (float)t;
}

/* Band arc a0..a1 drawn twice (rad, rad-1) so it reads as a heavier stroke. */
static void ui_dial_band(SDL_Renderer *r, float cx, float cy, float rad,
                         float a0, float a1, SDL_Color c) {
  ui_arc(r, cx, cy, rad, a0, a1, 24, c);
  ui_arc(r, cx, cy, rad - 1.0f, a0, a1, 24, c);
}

void ui_dial_gauge(SDL_Renderer *r, UiRect b, const UiTheme *th,
                   const char *label, double value, const char *unit,
                   int precision, UiRange range) {
  if (!ui_rect_valid(b)) {
    return;
  }
  UiStatus st = ui_status_for(value, range);
  float mx = b.x + b.w * 0.5f;

  /* caption row on top, digital readout row on the bottom, dial between */
  UiRect mid;
  UiRect lab = ui_split_top(b, kGlyph, 2.0f, &mid);
  UiRect readout = ui_split_bottom(mid, kGlyph, 2.0f, &mid);
  ui_text_center(r, mx, lab.y, th->text_dim, "%s", label);

  float cx = mid.x + mid.w * 0.5f;
  float cy = mid.y + mid.h * 0.5f;
  float rad = (mid.w < mid.h ? mid.w : mid.h) * 0.5f - 6.0f;
  if (rad < 8.0f) {
    /* too small for a dial -- fall back to just the readout */
    ui_text_center(r, mx, readout.y, ui_status_color(th, st), "%.*f %s",
                   precision, value, unit != NULL ? unit : "");
    return;
  }

  /* scale arc */
  ui_arc(r, cx, cy, rad, UI_DIAL_START, UI_DIAL_START + UI_DIAL_SWEEP, 64,
         th->frame);

  /* caution / warning band arcs, doubled up just inside the scale for weight.
   * High-side bands run outward from their threshold to the top of the scale;
   * low-side bands run inward from the bottom of the scale to their threshold.
   * The redder band is drawn last so it wins any overlap. */
  const float a_lo = UI_DIAL_START;
  const float a_hi = UI_DIAL_START + UI_DIAL_SWEEP;

  if (range.has_warn_hi) {
    float a1 = range.has_alert_hi ? ui_dial_angle(range.alert_hi, range) : a_hi;
    ui_dial_band(r, cx, cy, rad, ui_dial_angle(range.warn_hi, range), a1,
                 th->warn);
  }
  if (range.has_warn_lo) {
    float a0 = range.has_alert_lo ? ui_dial_angle(range.alert_lo, range) : a_lo;
    ui_dial_band(r, cx, cy, rad, a0, ui_dial_angle(range.warn_lo, range),
                 th->warn);
  }
  if (range.has_alert_hi) {
    ui_dial_band(r, cx, cy, rad, ui_dial_angle(range.alert_hi, range), a_hi,
                 th->alert);
  }
  if (range.has_alert_lo) {
    ui_dial_band(r, cx, cy, rad, a_lo, ui_dial_angle(range.alert_lo, range),
                 th->alert);
  }

  /* major tick marks */
  const int div = 5;
  for (int i = 0; i <= div; i++) {
    float a =
        (UI_DIAL_START + UI_DIAL_SWEEP * (float)i / (float)div) * UI_DIAL_D2R;
    float ca = SDL_cosf(a);
    float sa = SDL_sinf(a);
    ui_line(r, cx + ca * (rad - 5.0f), cy + sa * (rad - 5.0f), cx + ca * rad,
            cy + sa * rad, th->frame);
  }

  /* pointer */
  float na = ui_dial_angle(value, range) * UI_DIAL_D2R;
  SDL_Color needle = (st == UI_OK) ? th->text_bright : ui_status_color(th, st);
  ui_line(r, cx, cy, cx + SDL_cosf(na) * (rad - 4.0f),
          cy + SDL_sinf(na) * (rad - 4.0f), needle);
  ui_fill(r, (UiRect){cx - 2.0f, cy - 2.0f, 4.0f, 4.0f}, th->frame);

  /* digital readout in the gap at the bottom */
  ui_text_center(r, mx, readout.y, ui_status_color(th, st), "%.*f %s",
                 precision, value, unit != NULL ? unit : "");
}
