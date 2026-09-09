#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "platform/sdl/sdl_text.h"
#include "platform/sdl/sdl_time.h"
#include "platform/sdl/sdl_window.h"

typedef struct {
  SdlWindowContext window_ctx;
  SdlFrameTimer timer;
} AppState;

static AppState g_app_state;

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv) {
  (void)argc;
  (void)argv;

  *appstate = &g_app_state;

  if (!sdl_window_init(&g_app_state.window_ctx, "aero engine dt | simulator",
                       640, 480)) {
    return SDL_APP_FAILURE;
  }

  return SDL_APP_CONTINUE;
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  (void)appstate;

  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS; /* end the program, reporting success to the OS. */
  }
  return SDL_APP_CONTINUE; /* carry on with the program! */
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  AppState *state = (AppState *)appstate;
  SDL_Renderer *renderer = state->window_ctx.renderer;

  const float fps = sdl_time_tick(&state->timer);

  SDL_SetRenderDrawColor(renderer, 20, 40, 80, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  const SDL_Color white = {255, 255, 255, SDL_ALPHA_OPAQUE};
  const SDL_Color amber = {255, 200, 80, SDL_ALPHA_OPAQUE};

  const Uint64 ticks = SDL_GetTicks();
  const int ver = SDL_GetVersion();

  sdl_text_draw(renderer, SDL_TEXT_ANCHOR_TOP_LEFT, 0, white,
                "Aero Engine DT | Simulator Running");
  sdl_text_draw(renderer, SDL_TEXT_ANCHOR_TOP_LEFT, 1, white,
                "Time: %.2f seconds", (double)ticks / 1000.0);

  sdl_text_draw(renderer, SDL_TEXT_ANCHOR_TOP_RIGHT, 0, amber, "FPS: %.1f",
                (double)fps);

  sdl_text_draw(renderer, SDL_TEXT_ANCHOR_BOTTOM_LEFT, 0, white,
                "SDL %d.%d.%d", SDL_VERSIONNUM_MAJOR(ver),
                SDL_VERSIONNUM_MINOR(ver), SDL_VERSIONNUM_MICRO(ver));

  SDL_RenderPresent(renderer);

  return SDL_APP_CONTINUE;
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  (void)result;
  AppState *state = (AppState *)appstate;
  sdl_window_shutdown(&state->window_ctx);
}
