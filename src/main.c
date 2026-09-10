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
#include "telemetry/sensor.h"
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
  ModelState state;   /* exact model / twin state */
  ModelState display; /* noisy instrument feed the dashboard renders */
  Sensor sensor;
  int sensor_mode; /* 1 = show the noisy feed, 0 = show raw model state */
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

  SensorConfig scfg = sensor_config_default();
  sensor_init(&app->sensor, &scfg, 0xC0FFEEu);
  app->display = app->state;
  app->sensor_mode = 1;

  sdl_input_init(&app->input);
  dashboard_init(&app->dash);
  app->sample_accum_s = 0.0;

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  AppState *app = (AppState *)appstate;
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }
  if ((event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
       event->key.key == SDLK_M) ||
      (event->type == SDL_EVENT_GAMEPAD_BUTTON_DOWN &&
       event->gbutton.button == SDL_GAMEPAD_BUTTON_NORTH)) {
    app->sensor_mode = !app->sensor_mode;
  }
  sdl_input_handle_event(&app->input, event);
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

  /* Refresh the instrument feed + trend rings at a fixed cadence (not per
   * render frame), so the gauges read a lively ~10 Hz sensor sample rather
   * than fresh static every frame. */
  app->sample_accum_s += dt;
  while (app->sample_accum_s >= SAMPLE_PERIOD_S) {
    sensor_read_state(&app->sensor, &app->state, &app->display);
    dashboard_sample(&app->dash,
                     app->sensor_mode ? &app->display : &app->state);
    app->sample_accum_s -= SAMPLE_PERIOD_S;
  }

  const ModelState *shown = app->sensor_mode ? &app->display : &app->state;

  SDL_SetRenderDrawColor(renderer, 10, 14, 12, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  dashboard_draw(&app->dash, renderer, (float)WIN_W, (float)WIN_H, shown,
                 app->sync.engine_config.num_cylinders, app->input.throttle,
                 app->sync.sim_time_s, fps, app->sensor_mode);

  SDL_RenderPresent(renderer);
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  (void)result;
  AppState *app = (AppState *)appstate;
  sdl_input_shutdown(&app->input);
  sdl_window_shutdown(&app->window_ctx);
}
