#include "platform/sdl/sdl_window.h"

/* Set from the project VERSION in CMakeLists.txt. */
#ifndef AERO_VERSION
#define AERO_VERSION "dev"
#endif

bool sdl_window_init(SdlWindowContext *ctx, const char *title, int width,
                     int height) {
  SDL_SetAppMetadata("Aero Engine DT", AERO_VERSION,
                     "com.aeroenginedt.simulator");

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn\'t initialize SDL: %s", SDL_GetError());
    return false;
  }

  if (!SDL_CreateWindowAndRenderer(title, width, height, SDL_WINDOW_RESIZABLE,
                                   &ctx->window, &ctx->renderer)) {
    SDL_Log("Couldn\'t create window/renderer: %s", SDL_GetError());
    return false;
  }

  /* Title-bar / taskbar icon while running. Non-fatal if the PNG is missing. */
  const char *base = SDL_GetBasePath();
  if (base) {
    char path[1024];
    SDL_snprintf(path, sizeof path, "%sassets/icons/aero_engine_dt.png", base);
    SDL_Surface *icon = SDL_LoadPNG(path);
    if (icon) {
      SDL_SetWindowIcon(ctx->window, icon);
      SDL_DestroySurface(icon);
    } else {
      SDL_Log("window icon not loaded: %s", SDL_GetError());
    }
  }

  /* Pace the loop to the display instead of spinning a core flat out. */
  if (!SDL_SetRenderVSync(ctx->renderer, 1)) {
    SDL_Log("vsync unavailable: %s", SDL_GetError());
  }

  return true;
}

void sdl_window_shutdown(SdlWindowContext *ctx) {
  SDL_DestroyWindow(ctx->window);
  SDL_DestroyRenderer(ctx->renderer);

  ctx->window = NULL;
  ctx->renderer = NULL;
}
