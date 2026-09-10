#include "ui/ui_draw.h"

#include <stdarg.h>

#define UI_DRAW_BUF 256

static const float kGlyph = (float)SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;

double ui_clampd(double v, double lo, double hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

double ui_map(double v, double in_lo, double in_hi, double out_lo,
              double out_hi) {
  double span = in_hi - in_lo;
  if (span == 0.0) {
    return out_lo;
  }
  double t = (v - in_lo) / span;
  if (t < 0.0) {
    t = 0.0;
  }
  if (t > 1.0) {
    t = 1.0;
  }
  return out_lo + t * (out_hi - out_lo);
}

float ui_text_w(const char *s) {
  return s == NULL ? 0.0f : (float)SDL_strlen(s) * kGlyph;
}

/* Save current draw colour, set `c`; restore with ui_pop_color. */
static SDL_Color ui_push_color(SDL_Renderer *r, SDL_Color c) {
  SDL_Color prev = {0, 0, 0, 0};
  SDL_GetRenderDrawColor(r, &prev.r, &prev.g, &prev.b, &prev.a);
  SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
  return prev;
}

static void ui_pop_color(SDL_Renderer *r, SDL_Color prev) {
  SDL_SetRenderDrawColor(r, prev.r, prev.g, prev.b, prev.a);
}

void ui_fill(SDL_Renderer *r, UiRect rect, SDL_Color c) {
  SDL_Color prev = ui_push_color(r, c);
  SDL_FRect fr = {rect.x, rect.y, rect.w, rect.h};
  SDL_RenderFillRect(r, &fr);
  ui_pop_color(r, prev);
}

void ui_box(SDL_Renderer *r, UiRect rect, SDL_Color c) {
  SDL_Color prev = ui_push_color(r, c);
  SDL_FRect fr = {rect.x, rect.y, rect.w, rect.h};
  SDL_RenderRect(r, &fr);
  ui_pop_color(r, prev);
}

void ui_hline(SDL_Renderer *r, float x, float y, float len, SDL_Color c) {
  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderLine(r, x, y, x + len, y);
  ui_pop_color(r, prev);
}

void ui_vline(SDL_Renderer *r, float x, float y, float len, SDL_Color c) {
  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderLine(r, x, y, x, y + len);
  ui_pop_color(r, prev);
}

void ui_line(SDL_Renderer *r, float x0, float y0, float x1, float y1,
             SDL_Color c) {
  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderLine(r, x0, y0, x1, y1);
  ui_pop_color(r, prev);
}

#define UI_ARC_MAX_SEG 96
#define UI_DEG2RAD 0.017453292519943295f

void ui_arc(SDL_Renderer *r, float cx, float cy, float radius, float a0_deg,
            float a1_deg, int segments, SDL_Color c) {
  if (segments < 1 || radius <= 0.0f) {
    return;
  }
  if (segments > UI_ARC_MAX_SEG) {
    segments = UI_ARC_MAX_SEG;
  }
  SDL_FPoint pts[UI_ARC_MAX_SEG + 1];
  for (int i = 0; i <= segments; i++) {
    float a =
        (a0_deg + (a1_deg - a0_deg) * (float)i / (float)segments) * UI_DEG2RAD;
    pts[i].x = cx + SDL_cosf(a) * radius;
    pts[i].y = cy + SDL_sinf(a) * radius;
  }
  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderLines(r, pts, segments + 1);
  ui_pop_color(r, prev);
}

void ui_dot_leader(SDL_Renderer *r, float x0, float x1, float y, SDL_Color c) {
  if (x1 <= x0) {
    return;
  }
  SDL_Color prev = ui_push_color(r, c);
  for (float x = x0; x <= x1; x += 4.0f) {
    SDL_RenderPoint(r, x, y);
  }
  ui_pop_color(r, prev);
}

void ui_polyline(SDL_Renderer *r, const SDL_FPoint *pts, int n, SDL_Color c) {
  if (pts == NULL || n < 2) {
    return;
  }
  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderLines(r, pts, n);
  ui_pop_color(r, prev);
}

enum { UI_ALIGN_L, UI_ALIGN_R, UI_ALIGN_C };

static void ui_text_v(SDL_Renderer *r, float x, float y, SDL_Color c, int align,
                      const char *fmt, va_list ap) {
  char buf[UI_DRAW_BUF];
  SDL_vsnprintf(buf, sizeof(buf), fmt, ap);

  float w = (float)SDL_strlen(buf) * kGlyph;
  float draw_x = x;
  if (align == UI_ALIGN_R) {
    draw_x = x - w;
  } else if (align == UI_ALIGN_C) {
    draw_x = x - w * 0.5f;
  }

  SDL_Color prev = ui_push_color(r, c);
  SDL_RenderDebugText(r, SDL_floorf(draw_x), SDL_floorf(y), buf);
  ui_pop_color(r, prev);
}

void ui_text(SDL_Renderer *r, float x, float y, SDL_Color c, const char *fmt,
             ...) {
  va_list ap;
  va_start(ap, fmt);
  ui_text_v(r, x, y, c, UI_ALIGN_L, fmt, ap);
  va_end(ap);
}

void ui_text_right(SDL_Renderer *r, float x_right, float y, SDL_Color c,
                   const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  ui_text_v(r, x_right, y, c, UI_ALIGN_R, fmt, ap);
  va_end(ap);
}

void ui_text_center(SDL_Renderer *r, float x_center, float y, SDL_Color c,
                    const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  ui_text_v(r, x_center, y, c, UI_ALIGN_C, fmt, ap);
  va_end(ap);
}
