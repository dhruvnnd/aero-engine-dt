#include "platform/sdl/sdl_text.h"

#include <stdarg.h>

#define SDL_TEXT_BUF_SIZE 512

static const float kGlyph = (float)SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;

/*
 * Size of the coordinate space SDL_RenderDebugText draws into. When a logical
 * presentation is active that is the logical size; otherwise it is the output
 * size scaled back down by the render scale.
 */
static void sdl_text_canvas_size(SDL_Renderer *renderer, float *w, float *h) {
  int lw = 0;
  int lh = 0;
  SDL_RendererLogicalPresentation mode = SDL_LOGICAL_PRESENTATION_DISABLED;

  SDL_GetRenderLogicalPresentation(renderer, &lw, &lh, &mode);
  if (mode != SDL_LOGICAL_PRESENTATION_DISABLED && lw > 0 && lh > 0) {
    *w = (float)lw;
    *h = (float)lh;
    return;
  }

  int ow = 0;
  int oh = 0;
  SDL_GetRenderOutputSize(renderer, &ow, &oh);

  float sx = 1.0f;
  float sy = 1.0f;
  SDL_GetRenderScale(renderer, &sx, &sy);

  *w = (sx > 0.0f) ? (float)ow / sx : (float)ow;
  *h = (sy > 0.0f) ? (float)oh / sy : (float)oh;
}

static float sdl_text_anchor_x(SdlTextAnchor anchor, float canvas_w,
                               float text_w) {
  switch (anchor) {
  case SDL_TEXT_ANCHOR_TOP_LEFT:
  case SDL_TEXT_ANCHOR_BOTTOM_LEFT:
    return SDL_TEXT_MARGIN;
  case SDL_TEXT_ANCHOR_TOP_RIGHT:
  case SDL_TEXT_ANCHOR_BOTTOM_RIGHT:
    return canvas_w - SDL_TEXT_MARGIN - text_w;
  case SDL_TEXT_ANCHOR_TOP_CENTER:
  case SDL_TEXT_ANCHOR_BOTTOM_CENTER:
  case SDL_TEXT_ANCHOR_CENTER:
  default:
    return (canvas_w - text_w) * 0.5f;
  }
}

static float sdl_text_anchor_y(SdlTextAnchor anchor, float canvas_h, int row) {
  const float line_h = kGlyph + SDL_TEXT_LINE_LEADING;

  switch (anchor) {
  case SDL_TEXT_ANCHOR_TOP_LEFT:
  case SDL_TEXT_ANCHOR_TOP_CENTER:
  case SDL_TEXT_ANCHOR_TOP_RIGHT:
    return SDL_TEXT_MARGIN + (float)row * line_h;
  case SDL_TEXT_ANCHOR_BOTTOM_LEFT:
  case SDL_TEXT_ANCHOR_BOTTOM_CENTER:
  case SDL_TEXT_ANCHOR_BOTTOM_RIGHT:
    return canvas_h - SDL_TEXT_MARGIN - kGlyph - (float)row * line_h;
  case SDL_TEXT_ANCHOR_CENTER:
  default:
    return (canvas_h - kGlyph) * 0.5f + (float)row * line_h;
  }
}

static bool sdl_text_anchor_grows_up(SdlTextAnchor anchor) {
  return anchor == SDL_TEXT_ANCHOR_BOTTOM_LEFT ||
         anchor == SDL_TEXT_ANCHOR_BOTTOM_CENTER ||
         anchor == SDL_TEXT_ANCHOR_BOTTOM_RIGHT;
}

static int sdl_text_count_lines(const char *str) {
  int n = 1;
  for (const char *p = str; *p != '\0'; ++p) {
    if (*p == '\n') {
      ++n;
    }
  }
  return n;
}

/* Draws buf (which may contain '\n') anchored to an edge. Mutates buf. */
static void sdl_text_render_anchored(SDL_Renderer *renderer, SdlTextAnchor anchor,
                                     int start_line, SDL_Color color,
                                     char *buf) {
  float canvas_w = 0.0f;
  float canvas_h = 0.0f;
  sdl_text_canvas_size(renderer, &canvas_w, &canvas_h);

  Uint8 pr = 0;
  Uint8 pg = 0;
  Uint8 pb = 0;
  Uint8 pa = 0;
  SDL_GetRenderDrawColor(renderer, &pr, &pg, &pb, &pa);
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

  const int n_lines = sdl_text_count_lines(buf);
  const bool grows_up = sdl_text_anchor_grows_up(anchor);

  char *segment = buf;
  for (int i = 0; segment != NULL; ++i) {
    char *newline = SDL_strchr(segment, '\n');
    if (newline != NULL) {
      *newline = '\0';
    }

    /* Bottom anchors stack upward, so emit the last segment closest to the
     * edge to keep first-line-highest reading order. */
    const int row = grows_up ? start_line + (n_lines - 1 - i) : start_line + i;

    const float text_w = (float)SDL_strlen(segment) * kGlyph;
    const float x = sdl_text_anchor_x(anchor, canvas_w, text_w);
    const float y = sdl_text_anchor_y(anchor, canvas_h, row);

    SDL_RenderDebugText(renderer, SDL_floorf(x), SDL_floorf(y), segment);

    segment = (newline != NULL) ? newline + 1 : NULL;
  }

  SDL_SetRenderDrawColor(renderer, pr, pg, pb, pa);
}

void sdl_text_draw(SDL_Renderer *renderer, SdlTextAnchor anchor, int line,
                   SDL_Color color, const char *fmt, ...) {
  char buf[SDL_TEXT_BUF_SIZE];
  va_list ap;

  va_start(ap, fmt);
  SDL_vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  sdl_text_render_anchored(renderer, anchor, line, color, buf);
}

void sdl_text_draw_at(SDL_Renderer *renderer, float x, float y, SDL_Color color,
                      const char *fmt, ...) {
  char buf[SDL_TEXT_BUF_SIZE];
  va_list ap;

  va_start(ap, fmt);
  SDL_vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Uint8 pr = 0;
  Uint8 pg = 0;
  Uint8 pb = 0;
  Uint8 pa = 0;
  SDL_GetRenderDrawColor(renderer, &pr, &pg, &pb, &pa);
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

  const float line_h = kGlyph + SDL_TEXT_LINE_LEADING;
  float row_y = y;

  char *segment = buf;
  while (segment != NULL) {
    char *newline = SDL_strchr(segment, '\n');
    if (newline != NULL) {
      *newline = '\0';
    }

    SDL_RenderDebugText(renderer, SDL_floorf(x), SDL_floorf(row_y), segment);

    row_y += line_h;
    segment = (newline != NULL) ? newline + 1 : NULL;
  }

  SDL_SetRenderDrawColor(renderer, pr, pg, pb, pa);
}
