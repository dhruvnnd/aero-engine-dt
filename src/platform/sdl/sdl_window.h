#ifndef PLATFORM_SDL_SDL_WINDOWS_H
#define PLATFORM_SDL_SDL_WINDOWS_H

#include <SDL3/SDL.h>

typedef struct {
  SDL_Window *window;
  SDL_Renderer *renderer;
} SdlWindowContext;

bool sdl_window_init(SdlWindowContext *ctx, const char *title, int width,
                     int height);

void sdl_window_shutdown(SdlWindowContext *ctx);

#endif // !PLATFORM_SDL_SDL_WINDOWS_H
