#ifndef UI_UI_THEME_H
#define UI_UI_THEME_H

#include <SDL3/SDL.h>

#define UI_GLYPH_W 8.0f /* SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE */
#define UI_GLYPH_H 8.0f

typedef struct {
  SDL_Color bg;          /* screen / panel background (same: no nested fills) */
  SDL_Color frame;       /* box outlines, title rules */
  SDL_Color grid;        /* graticule lines, tick marks, dot leaders */
  SDL_Color text;        /* primary readout text */
  SDL_Color text_bright; /* emphasised values */
  SDL_Color text_dim;    /* labels, units, captions */
  SDL_Color ok;          /* nominal status token */
  SDL_Color warn;        /* caution band / token (amber) */
  SDL_Color alert;       /* warning band / token (red) */

  float pad;      /* inner margin between a box edge and its content */
  float gap;      /* gap between grid cells / stacked rows */
  float row_h;    /* height of one text readout row */
  float header_h; /* height reserved for a panel title + its rule */
} UiTheme;

/* Standard ground-station dark theme. */
static inline UiTheme ui_theme_default(void) {
  UiTheme t;
  t.bg = (SDL_Color){10, 14, 12, 255};
  t.frame = (SDL_Color){90, 104, 92, 255};
  t.grid = (SDL_Color){44, 54, 46, 255};
  t.text = (SDL_Color){176, 190, 176, 255};
  t.text_bright = (SDL_Color){224, 236, 224, 255};
  t.text_dim = (SDL_Color){112, 126, 112, 255};
  t.ok = (SDL_Color){128, 176, 120, 255};
  t.warn = (SDL_Color){206, 170, 78, 255};
  t.alert = (SDL_Color){210, 92, 78, 255};
  t.pad = 6.0f;
  t.gap = 6.0f;
  t.row_h = UI_GLYPH_H + 4.0f;
  t.header_h = UI_GLYPH_H + 8.0f;
  return t;
}

#endif /* UI_UI_THEME_H */
