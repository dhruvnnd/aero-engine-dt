#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include "model/state.h"
#include "model/sync.h"
#include "physics/engine_model.h"
#include "physics/environment.h"
#include "platform/sdl/sdl_input.h"
#include "platform/sdl/sdl_time.h"
#include "platform/sdl/sdl_window.h"
#include "ui/dashboard.h"

#define WIN_W 1000
#define WIN_H 680

/* Trend sampling cadence -- the histories advance at this rate regardless of
 * render frame rate. */
#define SAMPLE_PERIOD_S 0.1

/* External shaft load for the demo (prop + accessories), N*m */
#define DEMO_LOAD_NM 40.0

typedef struct {
  SdlWindowContext window_ctx;
  SdlFrameTimer timer;

  ModelSync sync;
  ModelState state;
  SdlInputState input;
  Dashboard dash;

  double ambient_c;
  double ambient_pressure_kpa;
  double sample_accum_s;
} AppState;

static AppState g_app_state;

SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv) {
  (void)argc;
  (void)argv;
  *appstate = &g_app_state;
  AppState *app = &g_app_state;

  if (!sdl_window_init(&app->window_ctx, "aero engine dt | dashboard", WIN_W,
                       WIN_H)) {
    return SDL_APP_FAILURE;
  }

  model_sync_init(&app->sync);

  AtmosphereState atm = environment_isa(0.0);
  app->ambient_c = atm.temperature_k - 273.15;
  app->ambient_pressure_kpa = atm.pressure_kpa;

  model_state_init(&app->state, &app->sync.engine_config, app->ambient_c);
  sdl_input_init(&app->input);
  dashboard_init(&app->dash);
  app->sample_accum_s = 0.0;

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  (void)appstate;
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
  AppState *app = (AppState *)appstate;
  SDL_Renderer *renderer = app->window_ctx.renderer;

  const float fps = sdl_time_tick(&app->timer);

  /* Frame delta, clamped so a stall doesn't launch the integrator. */
  double dt = (fps > 0.0f) ? (double)(1.0f / fps) : (1.0 / 60.0);
  if (dt > 0.05) {
    dt = 0.05;
  }

  sdl_input_update(&app->input, dt);

  EngineInput in;
  in.throttle = app->input.throttle;
  in.load_torque_nm = DEMO_LOAD_NM;
  in.ambient_pressure_kpa = app->ambient_pressure_kpa;
  model_sync_step(&app->sync, &app->state, &in, app->ambient_c, dt);

  app->sample_accum_s += dt;
  while (app->sample_accum_s >= SAMPLE_PERIOD_S) {
    dashboard_sample(&app->dash, &app->state);
    app->sample_accum_s -= SAMPLE_PERIOD_S;
  }

  SDL_SetRenderDrawColor(renderer, 10, 14, 12, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  dashboard_draw(&app->dash, renderer, (float)WIN_W, (float)WIN_H, &app->state,
                 app->sync.engine_config.num_cylinders, app->input.throttle,
                 app->sync.sim_time_s, fps);

  SDL_RenderPresent(renderer);
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  (void)result;
  AppState *app = (AppState *)appstate;
  sdl_window_shutdown(&app->window_ctx);
}
