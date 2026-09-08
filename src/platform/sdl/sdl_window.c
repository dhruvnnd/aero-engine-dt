#include "platform/sdl/sdl_window.h"

bool sdl_window_init(SdlWindowContext *ctx, const char *title, int width,
                     int height) {
  SDL_SetAppMetadata("Aero Engine DT", "0.1", "com.aeroenginedt.simulator");

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn\'t initialize SDL: %s", SDL_GetError());
    return false;
  }

  if (!SDL_CreateWindowAndRenderer(title, width, height, SDL_WINDOW_RESIZABLE,
                                   &ctx->window, &ctx->renderer)) {
    SDL_Log("Couldn\'t create window/renderer: %s", SDL_GetError());
    return false;
  }

  SDL_SetRenderLogicalPresentation(ctx->renderer, width, height,
                                   SDL_LOGICAL_PRESENTATION_LETTERBOX);

  return true;
}

void sdl_window_shutdown(SdlWindowContext *ctx) {
  SDL_DestroyWindow(ctx->window);
  SDL_DestroyRenderer(ctx->renderer);

  ctx->window = NULL;
  ctx->renderer = NULL;
}
