#include <SDL3/SDL_video.h>
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <string.h>

#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

#include "model/state.h"
#include "model/sync.h"
#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"
#include "physics/environment.h"
#include "platform/sdl/sdl_input.h"
#include "platform/sdl/sdl_time.h"
#include "platform/sdl/sdl_window.h"
#include "telemetry/event_log.h"
#include "telemetry/monitor.h"
#include "telemetry/sensor.h"
#include "telemetry/trends.h"
#include "ui/dashboard.h"
#include "ui/event_log_panel.h"
#include "ui/gamepad_panel.h"

#define WIN_W 1000
#define WIN_H 680

/* ImGui size multiplier */
#define UI_ZOOM 1.2f

/* Trend sampling cadence -- the histories advance at this rate regardless of
 * render frame rate. */
#define SAMPLE_PERIOD_S 0.1

/* External shaft load for the demo (prop + accessories), N*m -- a flat
 * placeholder until Phase 4's real propeller model exists (load should
 * scale with RPM/flight condition, not stay constant). 40.0 was calibrated
 * against the old mean-value torque curve; Phase 1's real geometry produces
 * only ~22 N*m at idle MAP (~30 kPa), so a fixed 40 N*m load stalled the
 * engine immediately at closed throttle every time. 8.0 leaves a comfortable
 * idle margin; it also means WOT revs higher than before, since the same
 * light load applies everywhere -- expected until Phase 4 fixes the whole
 * range at once. */
#define DEMO_LOAD_NM 8.0

typedef struct {
  SdlWindowContext window_ctx;
  bool show_gamepad;   /* ImGui gamepad window open/closed */
  bool show_event_log; /* ImGui event log window open/closed */
  SdlFrameTimer timer;

  ModelSync sync;
  ModelState state;   /* exact model / twin state */
  ModelState display; /* noisy instrument feed the dashboard renders */
  Sensor sensor;
  int sensor_mode; /* 1 = show the noisy feed, 0 = show raw model state */
  int fullscreen;
  SdlInputState input;
  Dashboard dash;
  FaultMonitor faults;
  Trends trends;
  EventLog events;

  double ambient_c;
  double ambient_pressure_kpa;
  double sample_accum_s;

  bool engine_stop_requested;
} AppState;

static AppState g_app_state;

SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv) {
  const char *engine_spec_path = NULL;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--engine-spec") && i + 1 < argc) {
      engine_spec_path = argv[++i];
    }
  }

  *appstate = &g_app_state;
  AppState *app = &g_app_state;

  if (!sdl_window_init(&app->window_ctx, "aero engine dt | dashboard", WIN_W,
                       WIN_H)) {
    return SDL_APP_FAILURE;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
#ifdef IMGUI_HAS_DOCK
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
  io.IniFilename = "aero_engine_dt_imgui.ini";
  ImGui::StyleColorsDark();
  const float ui_scale =
      SDL_GetWindowDisplayScale(app->window_ctx.window) * UI_ZOOM;
  ImGui::GetStyle().ScaleAllSizes(ui_scale);
  ImGui::GetStyle().FontScaleMain = ui_scale;
  ImGui_ImplSDL3_InitForSDLRenderer(app->window_ctx.window,
                                    app->window_ctx.renderer);
  ImGui_ImplSDLRenderer3_Init(app->window_ctx.renderer);

  app->show_gamepad = false; /* G toggles it */
  app->show_event_log = true; /* L toggles it */

  event_log_init(&app->events);
  event_log_push(&app->events, 0.0, EVENT_INFO, "SYSTEM", "dashboard started");

  model_sync_init(&app->sync);

  if (engine_spec_path) {
    EngineSpecResult r =
        engine_spec_load(engine_spec_path, &app->sync.engine_config);
    if (r.status == ENGINE_SPEC_ERR_PARSE) {
      SDL_Log("bad --engine-spec (parse error at line %d) -- aborting",
              r.error_line);
      event_log_push(&app->events, 0.0, EVENT_WARNING, "CONFIG",
                     "--engine-spec parse error at line %d -- aborting",
                     r.error_line);
      return SDL_APP_FAILURE;
    }
  }
  SDL_Log("engine config: %s",
          engine_spec_path ? engine_spec_path : "built-in default");
  event_log_push(&app->events, 0.0, EVENT_INFO, "CONFIG", "engine config: %s",
                 engine_spec_path ? engine_spec_path : "built-in default");

  AtmosphereState atm = environment_isa(0.0);
  app->ambient_c = atm.temperature_k - 273.15;
  app->ambient_pressure_kpa = atm.pressure_kpa;

  model_state_init(&app->state, &app->sync.engine_config, app->ambient_c);

  app->state.engine.run_state = ENGINE_STOPPED;
  app->state.engine.omega_rad_s = 0.0;
  app->state.engine.map_kpa = app->ambient_pressure_kpa; /* not being pumped,
                                                          * so no vacuum */
  event_log_push(&app->events, 0.0, EVENT_INFO, "ENGINE",
                 "engine off -- press I to start");

  SensorConfig scfg = sensor_config_default();
  sensor_init(&app->sensor, &scfg, 0xC0FFEEu);
  app->display = app->state;
  app->sensor_mode = 0;

  sdl_input_init(&app->input);
  dashboard_init(&app->dash);
  fault_monitor_init(&app->faults);
  trends_init(&app->trends);
  app->sample_accum_s = 0.0;

  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  AppState *app = (AppState *)appstate;
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS;
  }

  ImGui_ImplSDL3_ProcessEvent(event);
  const ImGuiIO &io = ImGui::GetIO();
  const bool is_key =
      event->type == SDL_EVENT_KEY_DOWN || event->type == SDL_EVENT_KEY_UP;
  const bool is_mouse = event->type == SDL_EVENT_MOUSE_MOTION ||
                        event->type == SDL_EVENT_MOUSE_BUTTON_DOWN ||
                        event->type == SDL_EVENT_MOUSE_BUTTON_UP ||
                        event->type == SDL_EVENT_MOUSE_WHEEL;
  if ((is_key && io.WantCaptureKeyboard) || (is_mouse && io.WantCaptureMouse)) {
    return SDL_APP_CONTINUE;
  }

  if (event->type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
    return SDL_APP_SUCCESS;
  }

  if ((event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
       event->key.key == SDLK_M) ||
      (event->type == SDL_EVENT_GAMEPAD_BUTTON_DOWN &&
       event->gbutton.button == SDL_GAMEPAD_BUTTON_NORTH)) {
    app->sensor_mode = !app->sensor_mode;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "MODE",
                   "display feed -> %s", app->sensor_mode ? "sensor" : "model");
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_G) {
    app->show_gamepad = !app->show_gamepad;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "PANEL",
                   "gamepad panel %s", app->show_gamepad ? "shown" : "hidden");
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_L) {
    app->show_event_log = !app->show_event_log;
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_I) {
    const double min_start_soc = 0.05;
    if (app->state.engine.run_state == ENGINE_STOPPED &&
        app->state.elec.batt_soc < min_start_soc) {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION,
                     "ENGINE", "battery too weak to start (SOC %.0f%%)",
                     app->state.elec.batt_soc * 100.0);
    } else {
      EngineRunState before = app->state.engine.run_state;
      engine_model_start(&app->state.engine, &app->sync.engine_config,
                         app->state.cyl);
      if (before == ENGINE_STOPPED) {
        event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                       "cranking...");
      }
    }
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_O) {
    if (app->state.engine.run_state == ENGINE_RUNNING) {
      app->engine_stop_requested = true;
      engine_model_stop(&app->state.engine);
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                     "ignition off");
    } else if (app->state.engine.run_state == ENGINE_CRANKING) {
      engine_model_stop(&app->state.engine);
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                     "cranking aborted");
    }
  }

  /* Handle fullscreen */
  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_F && app->window_ctx.window) {
    app->fullscreen = !app->fullscreen;
    SDL_SetWindowFullscreen(app->window_ctx.window, app->fullscreen);
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "DISPLAY",
                   "fullscreen %s", app->fullscreen ? "on" : "off");
  }

  if (event->type == SDL_EVENT_GAMEPAD_ADDED) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "INPUT",
                   "gamepad connected (id %u)", (unsigned)event->gdevice.which);
  }
  if (event->type == SDL_EVENT_GAMEPAD_REMOVED) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION, "INPUT",
                   "gamepad disconnected (id %u)",
                   (unsigned)event->gdevice.which);
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

  EnvInput env_in;
  env_in.altitude_m = app->input.altitude_m;
  env_in.airspeed_ms = app->input.airspeed_ms;
  env_in.oat_offset_c = app->input.oat_offset_c;
  AtmosphereState atm = environment_isa(env_in.altitude_m);

  EngineInput in;
  in.throttle = app->input.throttle;
  in.load_torque_nm = DEMO_LOAD_NM;
  in.ambient_pressure_kpa = atm.pressure_kpa;

  EngineRunState prev_run_state = app->state.engine.run_state;
  model_sync_step(&app->sync, &app->state, &in, &env_in, dt);
  EngineRunState cur_run_state = app->state.engine.run_state;
  if (prev_run_state != ENGINE_STOPPED && cur_run_state == ENGINE_STOPPED) {
    if (app->engine_stop_requested) {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                     "engine off -- press I to start");
      app->engine_stop_requested = false;
    } else {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING,
                     "ENGINE", "engine stalled -- press I to start");
    }
  } else if (prev_run_state == ENGINE_CRANKING &&
             cur_run_state == ENGINE_RUNNING) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                   "engine caught, running");
  }

  /* Refresh the instrument feed + trend rings at a fixed cadence (not per
   * render frame), so the gauges read a lively ~10 Hz sensor sample rather
   * than fresh static every frame. */
  app->sample_accum_s += dt;
  while (app->sample_accum_s >= SAMPLE_PERIOD_S) {
    sensor_read_state(&app->sensor, &app->state, &app->display);
    const ModelState *sampled = app->sensor_mode ? &app->display : &app->state;
    trends_sample(&app->trends, sampled);
    fault_monitor_check(&app->faults, &app->events, sampled,
                        app->sync.sim_time_s);
    app->sample_accum_s -= SAMPLE_PERIOD_S;
  }

  const ModelState *shown = app->sensor_mode ? &app->display : &app->state;

  ImGui_ImplSDLRenderer3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();
#ifdef IMGUI_HAS_DOCK
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(),
                               ImGuiDockNodeFlags_PassthruCentralNode);
#endif
  if (app->show_event_log) {
    event_log_panel_draw(&app->show_event_log, &app->events);
  }
  if (app->show_gamepad) {
    gamepad_panel_draw(&app->show_gamepad, &app->input);
  }
  ImGui::ShowDemoWindow();
  ImGui::Render();

  SDL_SetRenderDrawColor(renderer, 10, 14, 12, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  dashboard_draw(&app->dash, renderer, (float)WIN_W, (float)WIN_H, shown,
                 &app->trends, app->sync.engine_config.num_cylinders,
                 app->input.throttle, app->sync.sim_time_s, fps,
                 app->sensor_mode);

  /* disable logical presentation just for imgui */
  SDL_SetRenderLogicalPresentation(renderer, 0, 0,
                                   SDL_LOGICAL_PRESENTATION_DISABLED);
  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
  SDL_SetRenderLogicalPresentation(renderer, WIN_W, WIN_H,
                                   SDL_LOGICAL_PRESENTATION_LETTERBOX);
  SDL_RenderPresent(renderer);

  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  (void)result;
  AppState *app = (AppState *)appstate;
  sdl_input_shutdown(&app->input);
  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  sdl_window_shutdown(&app->window_ctx);
}
