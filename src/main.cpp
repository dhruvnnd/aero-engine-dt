#include <SDL3/SDL_video.h>
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <string.h>

#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"
#include "implot.h"

#include "data/run_recorder.h"
#include "model/sim_clock.h"
#include "model/state.h"
#include "model/sync.h"
#include "physics/engine_model.h"
#include "physics/engine_spec_io.h"
#include "physics/engine_trace.h"
#include "physics/environment.h"
#include "platform/sdl/sdl_input.h"
#include "platform/sdl/sdl_time.h"
#include "platform/sdl/sdl_window.h"
#include "telemetry/annunciator.h"
#include "telemetry/cyl_trends.h"
#include "telemetry/ecu_compare.h"
#include "telemetry/ecu_trends.h"
#include "telemetry/event_log.h"
#include "telemetry/monitor.h"
#include "telemetry/sensor.h"
#include "telemetry/trends.h"
#include "ui/alarm_strip.h"
#include "ui/controls_panel.h"
#include "ui/cyl_trends_panel.h"
#include "ui/ecu_compare_panel.h"
#include "ui/ecu_faults_panel.h"
#include "ui/ecu_io_panel.h"
#include "ui/ecu_panel.h"
#include "ui/ecu_trends_panel.h"
#include "ui/engine_spec_panel.h"
#include "ui/event_log_panel.h"
#include "ui/faults_panel.h"
#include "ui/gamepad_panel.h"
#include "ui/layouts.h"
#include "ui/readout_panels.h"
#include "ui/spec_editor_panel.h"
#include "ui/torque_trace_panel.h"
#include "ui/trends_panel.h"

/* Initial window size; shrunk to fit the display if it is too big. */
#define WIN_W 1680
#define WIN_H 1000

/* ImGui size multiplier */
#define UI_ZOOM 1.2f

/* Trend sampling cadence -- the histories advance at this rate regardless of
 * render frame rate. */
#define SAMPLE_PERIOD_S 0.1

/* Extra shaft load on top of the propeller (accessories, a dyno), N*m. The
 * propeller's own load comes from the engine spec's prop_* fields and the
 * flight condition, applied in model_sync_step(). */
#define ACCESSORY_LOAD_NM 0.0

typedef struct {
  SdlWindowContext window_ctx;
  PanelVisibility panels; /* which dockable panels are open */
  int layout_current;     /* LAYOUT_* last applied */
  int layout_pending;     /* LAYOUT_* to apply next frame, or -1 */
  bool layout_checked;    /* first-frame default-layout check done */
  bool show_implot_demo;
  bool show_imgui_demo;
  bool quit_requested;
  SdlFrameTimer timer;

  ModelSync sync;
  ModelState state;   /* exact model / twin state */
  ModelState display; /* noisy instrument feed the panels can show */
  Sensor sensor;
  int sensor_mode; /* 1 = show the noisy feed, 0 = show raw model state */
  int fullscreen;
  SdlInputState input;
  FaultMonitor faults;
  Annunciator ann;
  RunRecorder recorder;
  char spec_path[512];    /* engine spec in use; empty = built-in default */
  int logged_sensor_mode; /* display feed last written to the event log */
  SimClock clock;
  bool logged_paused; /* clock state last written to the event log */
  int logged_speed_idx;
  Trends trends;
  CylTrends cyl_trends;
  ModelSync shadow_sync;   /* the same engine with its ECU bypassed, for the ECU */
  ModelState shadow_state; /* Compare panel; stepped alongside the real one */
  EcuCompareTrends ecu_compare_trends;
  EcuTrends ecu_trends;      /* idle-governor loop history for the ECU panel */
  EcuIdleMode logged_ecu_mode; /* governor mode last checked for event-log lines */
  int logged_dtc_active[ECU_DTC_COUNT]; /* fault codes last written to the log */
  EcuSpeedSource logged_speed_source;   /* speed sensor last written to the log */
  EngineTrace engine_trace; /* sub-step torque/pressure trace for the Torque
                             * Ripple panel; attached to state.engine */
  EventLog events;

  double ambient_c;
  double ambient_pressure_kpa;
  double sample_accum_s;

  bool engine_stop_requested;
} AppState;

static AppState g_app_state;

/* Shrinks the window if it doesn't fit the display it opened on, then centres
 * it, so the large default size still works on smaller screens. */
static void fit_window_to_display(SDL_Window *window) {
  SDL_Rect usable;
  if (SDL_GetDisplayUsableBounds(SDL_GetDisplayForWindow(window), &usable)) {
    int w = 0;
    int h = 0;
    SDL_GetWindowSize(window, &w, &h);
    const int max_w = usable.w * 92 / 100;
    const int max_h = usable.h * 92 / 100;
    if (w > max_w || h > max_h) {
      SDL_SetWindowSize(window, w > max_w ? max_w : w, h > max_h ? max_h : h);
    }
  }
  SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}

static void acknowledge_alarms(AppState *app) {
  if (annunciator_unacked(&app->ann, CHANNEL_WARN) == 0 &&
      annunciator_unacked(&app->ann, CHANNEL_ALERT) == 0) {
    return;
  }
  annunciator_acknowledge(&app->ann);
  event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ALARM",
                 "alarms acknowledged");
}

/* File > Load engine spec: the dialog callback may run on another thread, so it
 * only hands the chosen path over; the main loop picks it up. */
static SDL_AtomicInt g_spec_pending;
static char g_spec_path[512];

static void SDLCALL on_spec_chosen(void *userdata, const char *const *filelist,
                                   int filter) {
  (void)userdata;
  (void)filter;
  if (filelist && filelist[0]) {
    SDL_strlcpy(g_spec_path, filelist[0], sizeof g_spec_path);
    SDL_SetAtomicInt(&g_spec_pending, 1);
  }
}

#define RECORD_DB_PATH "runs/dashboard.db"

static void engine_start(AppState *app) {
  const double min_start_soc = 0.05;
  if (app->state.engine.run_state == ENGINE_STOPPED &&
      app->state.elec.batt_soc < min_start_soc) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION, "ENGINE",
                   "battery too weak to start (SOC %.0f%%)",
                   app->state.elec.batt_soc * 100.0);
    return;
  }
  EngineRunState before = app->state.engine.run_state;
  engine_model_start(&app->state.engine, &app->sync.engine_config,
                     app->state.cyl);
  engine_model_start(&app->shadow_state.engine, &app->shadow_sync.engine_config,
                     app->shadow_state.cyl); /* no-op unless it is stopped */
  if (before == ENGINE_STOPPED) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                   "cranking...");
  }
}

static void engine_stop(AppState *app) {
  engine_model_stop(&app->shadow_state.engine);
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

/* The operator switch for the ECU's idle governor. */
static void idle_governor_toggle(AppState *app) {
  EcuState *ecu = &app->state.ecu;
  if (!ecu->fitted) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                   "no ECU fitted (ecu_fitted = 0 in the engine spec)");
    return;
  }
  if (ecu->idle_mode == ECU_IDLE_DISABLED) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                   "no idle governor configured (idle_target_rpm = 0)");
    return;
  }
  ecu_set_idle_enabled(ecu, !ecu->idle_enabled);
  if (ecu->idle_enabled) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                   "idle governor on");
  } else {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION, "ECU",
                   "idle governor off -- idle will sag to the engine's natural "
                   "speed");
  }
}

/* Injects (or clears) a fault on one of the ECU's speed sensors: 0 = crank
 * (primary), 1 = alternator (secondary). */
static void ecu_speed_fault_set(AppState *app, int channel, EcuFaultKind kind,
                                double value) {
  EcuSensorFault *f =
      channel == 0 ? &app->sync.ecu_rpm_fault : &app->sync.ecu_rpm2_fault;
  const char *name = channel == 0 ? "crank" : "alternator";
  const EcuFaultKind before = f->kind;
  ecu_sensor_fault_set(f, kind, value);
  if (kind == before) {
    return; /* just the value moved */
  }
  if (kind == ECU_FAULT_NONE) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                   "%s speed sensor fault cleared", name);
  } else {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "ECU",
                   "%s speed sensor fault injected: %s", name,
                   ecu_fault_kind_name(kind));
  }
}

/* The operator switch for the ECU's diagnostics and fallback. */
static void ecu_diagnostics_toggle(AppState *app) {
  EcuState *ecu = &app->state.ecu;
  if (!ecu->fitted) {
    return;
  }
  ecu_set_diag_enabled(ecu, !ecu->diag_enabled);
  if (ecu->diag_enabled) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                   "diagnostics on");
  } else {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION, "ECU",
                   "diagnostics off -- the ECU now trusts the crank sensor "
                   "without checking");
  }
}

/* Event-log lines for what the ECU's diagnostics did since the last frame: a
 * code setting or healing, and a change of the speed sensor it acts on. */
static void ecu_diag_log_changes(AppState *app) {
  const EcuState &e = app->state.ecu;
  if (!e.fitted || !e.diag_enabled) {
    return;
  }
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    const int now = e.diag.dtc[i].active;
    if (now == app->logged_dtc_active[i]) {
      continue;
    }
    const EcuDtcInfo *info = ecu_dtc_info((EcuDtcId)i);
    if (now) {
      event_log_push(&app->events, app->sync.sim_time_s,
                     info->severity ? EVENT_WARNING : EVENT_CAUTION, "ECU",
                     "fault %s: %s", info->code, info->description);
    } else {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                     "fault %s healed", info->code);
    }
    app->logged_dtc_active[i] = now;
  }
  if (e.speed_source != app->logged_speed_source) {
    if (e.speed_source == ECU_SRC_NONE) {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "ECU",
                     "no trusted speed sensor -- limp-home");
    } else {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                     "now using the %s speed sensor",
                     ecu_speed_source_name(e.speed_source));
    }
    app->logged_speed_source = e.speed_source;
  }
}

/* The shadow engine for the ECU Compare panel: the same engine and state, with
 * the ECU bypassed. */
static void shadow_resync(AppState *app) {
  app->shadow_sync = app->sync;
  app->shadow_sync.engine_config.ecu_fitted = 0;
  app->shadow_state = app->state;
  app->shadow_state.engine.trace = NULL; /* the trace belongs to the real one */
  app->shadow_state.ecu.fitted = 0;
}

/* Cold, stopped engine at sea level, with every monitor and trend cleared.
 * Keeps sync (config, faults, clock), input, sensor and the event log. */
static void reset_simulation_state(AppState *app) {
  AtmosphereState atm = environment_isa(0.0);
  app->ambient_c = atm.temperature_k - 273.15;
  app->ambient_pressure_kpa = atm.pressure_kpa;

  model_state_init(&app->state, &app->sync.engine_config, app->ambient_c);
  app->state.engine.run_state = ENGINE_STOPPED;
  app->state.engine.omega_rad_s = 0.0;
  app->state.engine.map_kpa = app->ambient_pressure_kpa; /* not being pumped,
                                                          * so no vacuum */
  engine_trace_clear(&app->engine_trace);
  app->state.engine.trace =
      &app->engine_trace; /* model_state_init() detached it */
  app->display = app->state;
  shadow_resync(app);
  app->engine_stop_requested = false;

  fault_monitor_init(&app->faults);
  annunciator_init(&app->ann);
  trends_init(&app->trends);
  cyl_trends_init(&app->cyl_trends);
  ecu_trends_init(&app->ecu_trends);
  ecu_compare_init(&app->ecu_compare_trends);
  app->logged_ecu_mode = app->state.ecu.idle_mode;
  for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
    app->logged_dtc_active[i] = 0;
  }
  app->logged_speed_source = ECU_SRC_PRIMARY;
  app->sample_accum_s = 0.0;

  event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ENGINE",
                 "engine off -- press I to start");
}

static void recording_stop(AppState *app, const char *why) {
  if (!run_recorder_active(&app->recorder)) {
    return;
  }
  const long rows = app->recorder.rows;
  /* Sync only uploads "completed" runs, and every way of ending a recording
   * here is deliberate, so the data is kept. */
  run_recorder_stop(&app->recorder, "completed");
  event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "RECORD",
                 "recording stopped (%s), %ld rows", why, rows);
}

static void recording_start(AppState *app) {
  RunLogMeta meta;
  memset(&meta, 0, sizeof meta);
  meta.source = "dashboard";
  meta.profile = NULL;
  meta.dt = SAMPLE_PERIOD_S;
  meta.duration_s = 0.0; /* open-ended */
  meta.load_nm = ACCESSORY_LOAD_NM;
  meta.seed = 0xC0FFEEu;
  meta.engine_spec = app->spec_path[0] ? app->spec_path : "default";
  meta.with_sensor = 1;

  if (run_recorder_start(&app->recorder, RECORD_DB_PATH, &meta,
                         app->sync.sim_time_s) != 0) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "RECORD",
                   "couldn't start recording into %s", RECORD_DB_PATH);
    return;
  }
  event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "RECORD",
                 "recording to %s (run %lld)", RECORD_DB_PATH,
                 (long long)app->recorder.run_id);
}

/* One row per sampling tick, from the exact model plus a noisy sensor reading.
 */
static void record_sample(AppState *app) {
  SensorReading reading = sensor_read(&app->sensor, &app->state);
  RunLogSample s = {};
  s.t = app->sync.sim_time_s;
  s.throttle = app->input.throttle;
  s.alt_m = app->input.altitude_m;
  s.ambient_c = app->state.env.oat_c;
  s.airspeed_ms = app->input.airspeed_ms;
  s.cool_index = environment_cool_index(
      app->state.env.density_kg_m3, app->state.env.airspeed_ms, app->state.rpm);
  s.state = &app->state;
  s.sensor = &reading;
  run_recorder_write(&app->recorder, &s);
}

/* Restarts the simulation with `cfg`. `label` names it (a spec file, or a
 * description); "" means the built-in default. Clears injected faults, trends
 * and alarms and ends any recording. */
static void apply_engine_config(AppState *app, const EngineConfig *cfg,
                                const char *label) {
  const EngineConfig c = *cfg; /* cfg may point into app->sync */
  char name[sizeof app->spec_path];
  snprintf(name, sizeof name, "%s",
           label ? label : ""); /* may alias spec_path */

  ModelSync fresh;
  model_sync_init(&fresh);
  if (name[0]) {
    /* an explicit engine: fuel / air flow follow its geometry */
    model_sync_apply_engine_config(&fresh, &c);
  } else {
    fresh.engine_config = c;
  }

  recording_stop(app, "simulation restarted");
  app->sync = fresh;
  memcpy(app->spec_path, name, sizeof name);
  reset_simulation_state(app);
  event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "CONFIG",
                 "engine config: %s", name[0] ? name : "built-in default");
}

/* Loads and validates the spec file at `path`, then restarts with it. Returns
 * false, changing nothing, if the file is unreadable, malformed or invalid. */
static bool load_engine_spec(AppState *app, const char *path) {
  EngineConfig cfg;
  const EngineSpecResult r = engine_spec_load(path, &cfg);
  if (r.status == ENGINE_SPEC_ERR_OPEN) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "CONFIG",
                   "couldn't open engine spec %s", path);
    return false;
  }
  if (r.status != ENGINE_SPEC_OK) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "CONFIG",
                   "engine spec parse error at line %d -- not loaded",
                   r.error_line);
    return false;
  }
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  const int issues = engine_config_check(&cfg, msgs, ENGINE_CONFIG_MAX_ISSUES);
  if (issues > 0) {
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_WARNING, "CONFIG",
                   "%s has %d issue(s), not loaded: %s", path, issues, msgs[0]);
    return false;
  }
  apply_engine_config(app, &cfg, path);
  return true;
}

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
  fit_window_to_display(app->window_ctx.window);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  /* No ImGuiConfigFlags_NavEnableKeyboard: with it, ImGui claims the keyboard
   * whenever any panel is focused, which swallows the sim hotkeys. */
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = "aero_engine_dt_imgui.ini";
  ImGui::StyleColorsDark();
  const float ui_scale =
      SDL_GetWindowDisplayScale(app->window_ctx.window) * UI_ZOOM;
  ImGui::GetStyle().ScaleAllSizes(ui_scale);
  ImGui::GetStyle().FontScaleMain = ui_scale;
  ImGui_ImplSDL3_InitForSDLRenderer(app->window_ctx.window,
                                    app->window_ctx.renderer);
  ImGui_ImplSDLRenderer3_Init(app->window_ctx.renderer);

  app->layout_current = LAYOUT_OVERVIEW;
  app->layout_pending = -1;
  app->panels = layout_visibility(LAYOUT_OVERVIEW);
  app->show_implot_demo = false;
  app->show_imgui_demo = false;

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
  if (engine_spec_path) {
    snprintf(app->spec_path, sizeof app->spec_path, "%s", engine_spec_path);
    model_sync_apply_engine_config(&app->sync, &app->sync.engine_config);
  }
  SDL_Log("engine config: %s",
          engine_spec_path ? engine_spec_path : "built-in default");
  event_log_push(&app->events, 0.0, EVENT_INFO, "CONFIG", "engine config: %s",
                 engine_spec_path ? engine_spec_path : "built-in default");

  SensorConfig scfg = sensor_config_default();
  sensor_init(&app->sensor, &scfg, 0xC0FFEEu);
  app->sensor_mode = 0;
  app->logged_sensor_mode = 0;

  sdl_input_init(&app->input);
  sim_clock_init(&app->clock);
  app->logged_paused = app->clock.paused;
  app->logged_speed_idx = app->clock.speed_idx;
  run_recorder_init(&app->recorder);
  reset_simulation_state(app);

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
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_G) {
    app->panels.gamepad = !app->panels.gamepad;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "PANEL",
                   "gamepad panel %s",
                   app->panels.gamepad ? "shown" : "hidden");
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_P) {
    app->clock.paused = !app->clock.paused;
  }
  if (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_PERIOD) {
    sim_clock_request_step(&app->clock);
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_L) {
    app->panels.event_log = !app->panels.event_log;
  }

  if ((event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
       event->key.key == SDLK_SPACE) ||
      (event->type == SDL_EVENT_GAMEPAD_BUTTON_DOWN &&
       event->gbutton.button == SDL_GAMEPAD_BUTTON_SOUTH)) {
    acknowledge_alarms(app);
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_I) {
    engine_start(app);
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_O) {
    engine_stop(app);
  }

  if (event->type == SDL_EVENT_KEY_DOWN && !event->key.repeat &&
      event->key.key == SDLK_K) {
    idle_governor_toggle(app);
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

  if (SDL_GetAtomicInt(&g_spec_pending)) {
    SDL_SetAtomicInt(&g_spec_pending, 0);
    load_engine_spec(app, g_spec_path);
  }

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
  in.load_torque_nm = ACCESSORY_LOAD_NM;
  in.ambient_pressure_kpa = atm.pressure_kpa;

  /* Sim time for this frame: scaled by speed, frozen while paused. Physics is
   * stepped in slices of at most SIM_CLOCK_MAX_STEP_S so fast-forward and slow
   * frames stay numerically tame. */
  const double sim_dt = sim_clock_advance(&app->clock, dt);
  EngineRunState prev_run_state = app->state.engine.run_state;
  double remaining = sim_dt;
  double h;
  while ((h = sim_clock_take_step(&remaining)) > 0.0) {
    model_sync_step(&app->sync, &app->state, &in, &env_in, h);
    if (app->sync.engine_config.ecu_fitted) {
      /* same faults, same inputs, no ECU */
      memcpy(app->shadow_sync.cyl_config, app->sync.cyl_config,
             sizeof app->shadow_sync.cyl_config);
      model_sync_step(&app->shadow_sync, &app->shadow_state, &in, &env_in, h);
    }
  }
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

  /* ECU events worth a log line: the governor running out of authority (and
   * recovering). The rest of its transitions follow the pilot's throttle and
   * would only add noise. */
  const EcuIdleMode ecu_mode = app->state.ecu.idle_mode;
  if (ecu_mode != app->logged_ecu_mode) {
    if (ecu_mode == ECU_IDLE_LIMITED) {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_CAUTION, "ECU",
                     "idle governor at its authority limit -- %.0f rpm below "
                     "the %.0f rpm target",
                     app->state.ecu.idle_error_rpm,
                     app->state.ecu.idle_target_rpm);
    } else if (app->logged_ecu_mode == ECU_IDLE_LIMITED &&
               ecu_mode == ECU_IDLE_ACTIVE) {
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                     "idle governor back within its authority");
    }
    app->logged_ecu_mode = ecu_mode;
  }
  ecu_diag_log_changes(app);

  /* Refresh the instrument feed + trend rings at a fixed cadence (not per
   * render frame), so the gauges read a lively ~10 Hz sensor sample rather
   * than fresh static every frame. */
  app->sample_accum_s += sim_dt;
  while (app->sample_accum_s >= SAMPLE_PERIOD_S) {
    sensor_read_state(&app->sensor, &app->state, &app->display);
    const ModelState *sampled = app->sensor_mode ? &app->display : &app->state;
    trends_sample(&app->trends, sampled);
    cyl_trends_sample(&app->cyl_trends, sampled,
                      app->sync.engine_config.num_cylinders);
    ecu_trends_sample(&app->ecu_trends, &app->state); /* the ECU's own numbers
                                                       * are exact, not sensed */
    if (app->sync.engine_config.ecu_fitted) {
      ecu_compare_sample(&app->ecu_compare_trends, &app->state,
                         &app->shadow_state);
    }
    fault_monitor_check(&app->faults, &app->events, sampled,
                        app->sync.sim_time_s);
    annunciator_update(&app->ann, sampled);
    if (run_recorder_active(&app->recorder)) {
      record_sample(app);
    }
    app->sample_accum_s -= SAMPLE_PERIOD_S;
  }

  const ModelState *shown = app->sensor_mode ? &app->display : &app->state;

  ImGui_ImplSDLRenderer3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();

  /* Settings (and any saved dock tree) are loaded by now. With none for our
   * dockspace -- first run, or an old .ini -- start from the default layout. */
  if (!app->layout_checked) {
    app->layout_checked = true;
    if (!layout_dockspace_exists()) {
      app->layout_pending = LAYOUT_OVERVIEW;
    }
  }
  /* Rebuilding must happen before the dockspace is submitted this frame. */
  if (app->layout_pending >= 0) {
    layout_apply(app->layout_pending, ImGui::GetMainViewport()->WorkSize);
    app->panels = layout_visibility(app->layout_pending);
    app->layout_current = app->layout_pending;
    app->layout_pending = -1;
  }
  ImGui::DockSpaceOverViewport(layout_dockspace_id(), ImGui::GetMainViewport(),
                               ImGuiDockNodeFlags_None);

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      const bool recording = run_recorder_active(&app->recorder);
      if (ImGui::MenuItem(recording ? "Stop recording" : "Start recording")) {
        if (recording) {
          recording_stop(app, "stopped by user");
        } else {
          recording_start(app);
        }
      }
      if (recording) {
        ImGui::TextDisabled("  %s  %ld rows", RECORD_DB_PATH,
                            app->recorder.rows);
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Load engine spec...")) {
        static const SDL_DialogFileFilter filters[] = {
            {"Engine spec (*.cfg)", "cfg"}, {"All files", "*"}};
        SDL_ShowOpenFileDialog(on_spec_chosen, NULL, app->window_ctx.window,
                               filters, 2, NULL, false);
      }
      if (ImGui::MenuItem("Restart simulation")) {
        apply_engine_config(app, &app->sync.engine_config, app->spec_path);
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Cold-start the engine. Clears injected faults,\n"
                          "trends and alarms, and ends any recording.");
      }
      ImGui::TextDisabled("  engine: %s", app->spec_path[0]
                                              ? app->spec_path
                                              : "built-in default");
      ImGui::Separator();
      if (ImGui::MenuItem("Quit", "Alt+F4")) {
        app->quit_requested = true;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Layout")) {
      for (int i = 0; i < LAYOUT_COUNT; i++) {
        if (ImGui::MenuItem(layout_name(i), NULL, app->layout_current == i)) {
          app->layout_pending = i;
        }
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Reset current layout")) {
        app->layout_pending = app->layout_current;
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      ImGui::MenuItem("Alarms", NULL, &app->panels.alarms);
      ImGui::MenuItem("Sim", NULL, &app->panels.sim);
      ImGui::MenuItem("Controls", NULL, &app->panels.controls);
      ImGui::MenuItem("Fault Injection", NULL, &app->panels.faults);
      ImGui::MenuItem("Engine Spec", NULL, &app->panels.engine_spec);
      ImGui::MenuItem("Spec Editor", NULL, &app->panels.spec_editor);
      ImGui::MenuItem("Instruments", NULL, &app->panels.instruments);
      ImGui::MenuItem("Environment", NULL, &app->panels.environment);
      ImGui::MenuItem("Cylinders", NULL, &app->panels.cylinders);
      ImGui::MenuItem("Trends", NULL, &app->panels.trends);
      ImGui::MenuItem("Cylinder Trends", NULL, &app->panels.cyl_trends);
      ImGui::MenuItem("Torque Ripple", NULL, &app->panels.torque_trace);
      ImGui::MenuItem("ECU", NULL, &app->panels.ecu);
      ImGui::MenuItem("ECU Trends", NULL, &app->panels.ecu_trends);
      ImGui::MenuItem("ECU I/O", NULL, &app->panels.ecu_io);
      ImGui::MenuItem("ECU Faults", NULL, &app->panels.ecu_faults);
      ImGui::MenuItem("ECU Compare", NULL, &app->panels.ecu_compare);
      ImGui::MenuItem("Event Log", "L", &app->panels.event_log);
      ImGui::MenuItem("Gamepad", "G", &app->panels.gamepad);
      ImGui::Separator();
      ImGui::MenuItem("ImGui demo", NULL, &app->show_imgui_demo);
      ImGui::MenuItem("ImPlot demo", NULL, &app->show_implot_demo);
      ImGui::EndMenu();
    }
    if (run_recorder_active(&app->recorder)) {
      ImGui::TextColored(ImVec4(0.90f, 0.22f, 0.20f, 1.0f), "  REC %ld",
                         app->recorder.rows);
    }
    if (!app->panels.alarms) {
      alarm_menu_indicator(&app->ann);
    }
    ImGui::EndMainMenuBar();
  }

  if (app->panels.alarms && alarm_strip_draw(&app->panels.alarms, &app->ann)) {
    acknowledge_alarms(app);
  }

  if (app->panels.sim) {
    sim_panel_draw(&app->panels.sim, shown, app->sync.sim_time_s,
                   app->input.throttle, fps, app->sensor_mode != 0,
                   &app->clock);
  }
  if (app->panels.engine_spec) {
    engine_spec_panel_draw(&app->panels.engine_spec, &app->sync,
                           app->spec_path);
  }
  if (app->panels.spec_editor) {
    const SpecEditorResult ed = spec_editor_panel_draw(
        &app->panels.spec_editor, &app->sync.engine_config, app->spec_path,
        app->window_ctx.window, &app->events, app->sync.sim_time_s);
    if (ed.apply) {
      apply_engine_config(app, &ed.config, ed.name);
    }
  }
  if (app->panels.controls) {
    const ControlActions act = controls_panel_draw(
        &app->panels.controls, &app->input, shown, &app->sensor_mode);
    if (act.start_engine) {
      engine_start(app);
    }
    if (act.stop_engine) {
      engine_stop(app);
    }
    if (act.toggle_idle_governor) {
      idle_governor_toggle(app);
    }
  }
  if (app->panels.faults) {
    faults_panel_draw(&app->panels.faults, app->sync.cyl_config,
                      app->sync.engine_config.num_cylinders, &app->events,
                      app->sync.sim_time_s);
  }
  if (app->panels.instruments) {
    instruments_panel_draw(&app->panels.instruments, shown);
  }
  if (app->panels.environment) {
    environment_panel_draw(&app->panels.environment, shown);
  }
  if (app->panels.cylinders) {
    cylinders_panel_draw(&app->panels.cylinders, shown,
                         app->sync.engine_config.num_cylinders);
  }
  if (app->panels.trends) {
    trends_panel_draw(&app->panels.trends, &app->trends, SAMPLE_PERIOD_S);
  }
  if (app->panels.cyl_trends) {
    cyl_trends_panel_draw(&app->panels.cyl_trends, &app->cyl_trends,
                          app->sync.engine_config.num_cylinders,
                          SAMPLE_PERIOD_S);
  }
  if (app->panels.ecu) {
    const EcuActions ea =
        ecu_panel_draw(&app->panels.ecu, &app->state, &app->sync.engine_config);
    if (ea.toggle_idle_governor) {
      idle_governor_toggle(app);
    }
  }
  if (app->panels.ecu_io) {
    const EcuSensorFault faults[2] = {app->sync.ecu_rpm_fault,
                                      app->sync.ecu_rpm2_fault};
    const EcuIoActions ia =
        ecu_io_panel_draw(&app->panels.ecu_io, &app->state, faults);
    if (ia.set_fault) {
      ecu_speed_fault_set(app, ia.channel, ia.kind, ia.value);
    }
  }
  if (app->panels.ecu_faults) {
    const EcuFaultsActions fa =
        ecu_faults_panel_draw(&app->panels.ecu_faults, &app->state);
    if (fa.toggle_diagnostics) {
      ecu_diagnostics_toggle(app);
    }
    if (fa.clear_codes) {
      ecu_clear_codes(&app->state.ecu);
      for (int i = 0; i < (int)ECU_DTC_COUNT; i++) {
        app->logged_dtc_active[i] = 0; /* a fault still present logs again */
      }
      event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "ECU",
                     "fault codes cleared");
    }
  }
  if (app->panels.ecu_compare) {
    if (ecu_compare_panel_draw(&app->panels.ecu_compare, &app->state,
                               &app->shadow_state, &app->ecu_compare_trends,
                               app->sync.engine_config.ecu_fitted != 0,
                               SAMPLE_PERIOD_S)) {
      shadow_resync(app);
    }
  }
  if (app->panels.ecu_trends) {
    ecu_trends_panel_draw(&app->panels.ecu_trends, &app->ecu_trends,
                          &app->sync.engine_config,
                          app->sync.engine_config.ecu_fitted != 0,
                          SAMPLE_PERIOD_S);
  }
  if (app->panels.torque_trace) {
    torque_trace_panel_draw(&app->panels.torque_trace, &app->engine_trace,
                            &app->sync.engine_config);
  }
  if (app->panels.event_log) {
    event_log_panel_draw(&app->panels.event_log, &app->events);
  }
  if (app->panels.gamepad) {
    gamepad_panel_draw(&app->panels.gamepad, &app->input);
  }
  if (app->show_imgui_demo) {
    ImGui::ShowDemoWindow(&app->show_imgui_demo);
  }
  if (app->show_implot_demo) {
    ImPlot::ShowDemoWindow(&app->show_implot_demo);
  }
  /* Pause / speed can change from the panel or a hotkey; log it either way. */
  if (app->clock.paused != app->logged_paused) {
    app->logged_paused = app->clock.paused;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "SIM",
                   "simulation %s", app->clock.paused ? "paused" : "resumed");
  }
  if (app->sensor_mode != app->logged_sensor_mode) {
    app->logged_sensor_mode = app->sensor_mode;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "MODE",
                   "display feed -> %s", app->sensor_mode ? "sensor" : "model");
  }
  if (app->clock.speed_idx != app->logged_speed_idx) {
    app->logged_speed_idx = app->clock.speed_idx;
    event_log_push(&app->events, app->sync.sim_time_s, EVENT_INFO, "SIM",
                   "sim speed %.2gx", sim_clock_speed(&app->clock));
  }

  ImGui::Render();

  SDL_SetRenderDrawColor(renderer, 10, 14, 12, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
  SDL_RenderPresent(renderer);

  return app->quit_requested ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  (void)result;
  AppState *app = (AppState *)appstate;
  recording_stop(app, "application closed");
  sdl_input_shutdown(&app->input);
  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  sdl_window_shutdown(&app->window_ctx);
}
