#ifndef UI_UI_DRAW_H
#define UI_UI_DRAW_H

#include <SDL3/SDL.h>

#include "ui/ui_layout.h"

double ui_clampd(double v, double lo, double hi);

/* Linear map of `v` from [in_lo, in_hi] onto [out_lo, out_hi], clamped to the
 * output range. Returns out_lo if the input range is degenerate. */
double ui_map(double v, double in_lo, double in_hi, double out_lo,
              double out_hi);

/* Pixel width of `s` in the debug font. */
float ui_text_w(const char *s);

void ui_fill(SDL_Renderer *r, UiRect rect, SDL_Color c);
void ui_box(SDL_Renderer *r, UiRect rect, SDL_Color c); /* 1px outline */
void ui_hline(SDL_Renderer *r, float x, float y, float len, SDL_Color c);
void ui_vline(SDL_Renderer *r, float x, float y, float len, SDL_Color c);
void ui_line(SDL_Renderer *r, float x0, float y0, float x1, float y1,
             SDL_Color c);

/* Circular arc centred at (cx, cy), swept from a0 to a1 (degrees, screen
 * convention: 0 = +x / 3 o'clock, +90 = +y / 6 o'clock), approximated by
 * `segments` straight chords. */
void ui_arc(SDL_Renderer *r, float cx, float cy, float radius, float a0_deg,
            float a1_deg, int segments, SDL_Color c);

/* Row of dots from x0 to x1 at baseline `y`, every 4px -- for "label ... value"
 * leaders. */
void ui_dot_leader(SDL_Renderer *r, float x0, float x1, float y, SDL_Color c);

/* Connected line through `pts` (screen coords already computed by caller). */
void ui_polyline(SDL_Renderer *r, const SDL_FPoint *pts, int n, SDL_Color c);

void ui_text(SDL_Renderer *r, float x, float y, SDL_Color c,
             SDL_PRINTF_FORMAT_STRING const char *fmt, ...)
    SDL_PRINTF_VARARG_FUNC(5);

/* Right edge of the string sits at `x_right`. */
void ui_text_right(SDL_Renderer *r, float x_right, float y, SDL_Color c,
                   SDL_PRINTF_FORMAT_STRING const char *fmt, ...)
    SDL_PRINTF_VARARG_FUNC(5);

/* String centred on `x_center`. */
void ui_text_center(SDL_Renderer *r, float x_center, float y, SDL_Color c,
                    SDL_PRINTF_FORMAT_STRING const char *fmt, ...)
    SDL_PRINTF_VARARG_FUNC(5);

#endif /* UI_UI_DRAW_H */
