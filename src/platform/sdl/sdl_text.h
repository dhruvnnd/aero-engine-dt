#ifndef PLATFORM_SDL_SDL_TEXT_H
#define PLATFORM_SDL_SDL_TEXT_H

#include <SDL3/SDL.h>

#define SDL_TEXT_MARGIN 8.0f
#define SDL_TEXT_LINE_LEADING 2.0f

typedef enum {
  SDL_TEXT_ANCHOR_TOP_LEFT,
  SDL_TEXT_ANCHOR_TOP_CENTER,
  SDL_TEXT_ANCHOR_TOP_RIGHT,
  SDL_TEXT_ANCHOR_BOTTOM_LEFT,
  SDL_TEXT_ANCHOR_BOTTOM_CENTER,
  SDL_TEXT_ANCHOR_BOTTOM_RIGHT,
  SDL_TEXT_ANCHOR_CENTER
} SdlTextAnchor;

/*
 * Draw a formatted line anchored to a screen edge/corner.
 *
 *   line  - row index away from the anchored edge (0 = flush against it,
 *           1 = one row further in, ...). Lets several readouts share a
 *           corner without the caller doing offset math.
 */
void sdl_text_draw(SDL_Renderer *renderer, SdlTextAnchor anchor, int line,
                   SDL_Color color, SDL_PRINTF_FORMAT_STRING const char *fmt,
                   ...) SDL_PRINTF_VARARG_FUNC(5);

/*
 * Draw formatted text with its top-left corner at an explicit logical
 * coordinate. Same colour save/restore and '\n' handling as sdl_text_draw.
 */
void sdl_text_draw_at(SDL_Renderer *renderer, float x, float y, SDL_Color color,
                      SDL_PRINTF_FORMAT_STRING const char *fmt, ...)
    SDL_PRINTF_VARARG_FUNC(5);

#endif // !PLATFORM_SDL_SDL_TEXT_H
