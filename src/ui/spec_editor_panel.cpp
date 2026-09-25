#include "ui/spec_editor_panel.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imgui.h"
#include "physics/engine_spec_io.h"
#include "ui/panel_names.h"

static const ImVec4 COL_CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

/* ---- editable fields --------------------------------------------------- */

struct Field {
  const char *label; /* with unit */
  const char *key;   /* spec-file key */
  int group;
  double *(*get)(EngineConfig &);
  double lo, hi;         /* slider extent (typed values are not clamped) */
  double typ_lo, typ_hi; /* typical range; none when typ_hi <= typ_lo */
  const char *fmt;
};

#define GET(expr)                                                              \
  [](EngineConfig &c) -> double * { return &(expr); }

static const char *const GROUPS[] = {"DYNAMICS", "GEOMETRY", "COMBUSTION"};

/* Typical ranges are the ones documented in engine_spec_io.c. */
static const Field FIELDS[] = {
    {"crank inertia (kg*m^2)", "inertia_kg_m2", 0, GET(c.inertia_kg_m2), 0.05,
     3.0, 0.3, 1.0, "%.3f"},
    {"MAP time constant (s)", "map_tau_s", 0, GET(c.map_tau_s), 0.02, 1.0, 0.1,
     0.4, "%.3f"},
    {"friction (N*m/(rad/s))", "friction_coeff_nm_per_rad_s", 0,
     GET(c.friction_coeff_nm_per_rad_s), 0.005, 0.5, 0.05, 0.2, "%.3f"},
    {"starter torque (N*m)", "starter_torque_nm", 0, GET(c.starter_torque_nm),
     5.0, 100.0, 0.0, 0.0, "%.1f"},
    {"starter catch (rpm)", "starter_catch_rpm", 0, GET(c.starter_catch_rpm),
     200.0, 1500.0, 0.0, 0.0, "%.0f"},

    {"bore (m)", "bore_m", 1, GET(c.geom.bore_m), 0.04, 0.14, 0.07, 0.10,
     "%.4f"},
    {"stroke (m)", "stroke_m", 1, GET(c.geom.stroke_m), 0.04, 0.14, 0.07, 0.10,
     "%.4f"},
    {"conrod length (m)", "conrod_len_m", 1, GET(c.geom.conrod_len_m), 0.08,
     0.25, 0.12, 0.20, "%.4f"},
    {"compression ratio", "compression_ratio", 1,
     GET(c.geom.compression_ratio), 5.0, 15.0, 8.5, 11.0, "%.2f"},
    {"exhaust valve opens (deg)", "evo_deg", 1, GET(c.geom.evo_deg), 60.0,
     180.0, 110.0, 150.0, "%.0f"},
    {"intake valve closes (deg)", "ivc_deg", 1, GET(c.geom.ivc_deg), 500.0,
     700.0, 570.0, 610.0, "%.0f"},
    {"reciprocating mass (kg)", "m_recip_kg", 1, GET(c.geom.m_recip_kg), 0.1,
     1.0, 0.3, 0.6, "%.3f"},

    {"Wiebe a", "wiebe_a", 2, GET(c.geom.wiebe_a), 1.0, 10.0, 3.0, 6.0,
     "%.2f"},
    {"Wiebe m", "wiebe_m", 2, GET(c.geom.wiebe_m), 0.5, 4.0, 2.0, 3.0, "%.2f"},
    {"burn duration (deg)", "delta_theta_burn_deg", 2,
     GET(c.geom.delta_theta_burn_deg), 20.0, 90.0, 40.0, 60.0, "%.0f"},
    {"spark base (deg BTDC)", "spark_base_btdc_deg", 2,
     GET(c.geom.spark_base_btdc_deg), 0.0, 30.0, 5.0, 20.0, "%.1f"},
    {"spark rpm gain (deg/1000rpm)", "spark_rpm_gain_deg_per_1000rpm", 2,
     GET(c.geom.spark_rpm_gain_deg_per_1000rpm), 0.0, 12.0, 3.0, 8.0, "%.2f"},
    {"spark MAP retard (deg/kPa)", "spark_map_retard_deg_per_kpa", 2,
     GET(c.geom.spark_map_retard_deg_per_kpa), 0.0, 0.5, 0.05, 0.25, "%.3f"},
    {"combustion efficiency", "combustion_efficiency", 2,
     GET(c.geom.combustion_efficiency), 0.05, 1.0, 0.25, 0.35, "%.3f"},
};
static const int FIELD_COUNT = (int)(sizeof FIELDS / sizeof FIELDS[0]);

static bool config_equal(const EngineConfig &a, const EngineConfig &b) {
  if (a.num_cylinders != b.num_cylinders) {
    return false;
  }
  for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
    if (a.firing_order[i] != b.firing_order[i]) {
      return false;
    }
  }
  EngineConfig ac = a;
  EngineConfig bc = b;
  for (int i = 0; i < FIELD_COUNT; i++) {
    if (*FIELDS[i].get(ac) != *FIELDS[i].get(bc)) {
      return false;
    }
  }
  return true;
}

/* ---- state and file dialogs ------------------------------------------- */

struct EditorState {
  bool init = false;
  EngineConfig draft;
  EngineConfig baseline; /* what the draft was last loaded / saved as */
  char origin[256] = "";
};

static EditorState g;

/* Dialog callbacks can run on another thread: they only hand over a path. */
static SDL_AtomicInt g_open_pending;
static SDL_AtomicInt g_save_pending;
static char g_open_path[512];
static char g_save_path[512];

static void SDLCALL on_open_chosen(void *, const char *const *files, int) {
  if (files && files[0]) {
    SDL_strlcpy(g_open_path, files[0], sizeof g_open_path);
    SDL_SetAtomicInt(&g_open_pending, 1);
  }
}

static void SDLCALL on_save_chosen(void *, const char *const *files, int) {
  if (files && files[0]) {
    SDL_strlcpy(g_save_path, files[0], sizeof g_save_path);
    SDL_SetAtomicInt(&g_save_pending, 1);
  }
}

static const SDL_DialogFileFilter SPEC_FILTERS[] = {
    {"Engine spec (*.cfg)", "cfg"}, {"All files", "*"}};

static void set_origin(const char *name) {
  snprintf(g.origin, sizeof g.origin, "%s", name);
}

static void handle_dialog_results(EventLog *log, double t) {
  if (SDL_GetAtomicInt(&g_open_pending)) {
    SDL_SetAtomicInt(&g_open_pending, 0);
    EngineConfig cfg;
    const EngineSpecResult r = engine_spec_load(g_open_path, &cfg);
    if (r.status == ENGINE_SPEC_ERR_OPEN) {
      event_log_push(log, t, EVENT_WARNING, "SPEC", "couldn't open %s",
                     g_open_path);
    } else if (r.status != ENGINE_SPEC_OK) {
      event_log_push(log, t, EVENT_WARNING, "SPEC",
                     "%s: parse error at line %d -- not loaded", g_open_path,
                     r.error_line);
    } else {
      g.draft = cfg;
      g.baseline = cfg;
      set_origin(g_open_path);
      event_log_push(log, t, EVENT_INFO, "SPEC", "editor loaded %s%s",
                     g_open_path,
                     r.unknown_keys ? " (unknown keys ignored)" : "");
    }
  }

  if (SDL_GetAtomicInt(&g_save_pending)) {
    SDL_SetAtomicInt(&g_save_pending, 0);
    char path[520];
    snprintf(path, sizeof path, "%s", g_save_path);
    const char *dot = strrchr(path, '.');
    const char *slash = strrchr(path, '/');
    const char *bslash = strrchr(path, '\\');
    if (bslash && (!slash || bslash > slash)) {
      slash = bslash;
    }
    if (!dot || (slash && dot < slash)) {
      strncat(path, ".cfg", sizeof path - strlen(path) - 1);
    }

    if (engine_config_check(&g.draft, NULL, 0) > 0) {
      event_log_push(log, t, EVENT_WARNING, "SPEC",
                     "not saved: the draft has validation issues");
    } else if (engine_spec_save(path, &g.draft) != 0) {
      event_log_push(log, t, EVENT_WARNING, "SPEC", "couldn't write %s", path);
    } else {
      g.baseline = g.draft;
      set_origin(path);
      event_log_push(log, t, EVENT_INFO, "SPEC", "saved %s", path);
    }
  }
}

/* ---- small layout helpers --------------------------------------------- */

static void wrap_next(float next_width) {
  const ImGuiStyle &style = ImGui::GetStyle();
  const float right =
      ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - style.WindowPadding.x;
  if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + next_width < right) {
    ImGui::SameLine();
  }
}

static float button_width(const char *label) {
  return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
}

static void derived_row(const char *label, double v, const char *unit) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  ImGui::Text("%.4g %s", v, unit);
}

/* ---- panel ------------------------------------------------------------- */

SpecEditorResult spec_editor_panel_draw(bool *open, const EngineConfig *running,
                                        const char *running_name,
                                        SDL_Window *window, EventLog *log,
                                        double sim_time_s) {
  SpecEditorResult res;
  memset(&res, 0, sizeof res);

  if (!g.init) {
    g.draft = *running;
    g.baseline = *running;
    set_origin(running_name && running_name[0] ? running_name
                                               : "built-in default");
    g.init = true;
  }
  handle_dialog_results(log, sim_time_s);

  ImGui::SetNextWindowSize(ImVec2(520.0f, 720.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_SPEC_EDITOR, open)) {
    ImGui::End();
    return res;
  }

  const bool dirty = !config_equal(g.draft, g.baseline);
  ImGui::TextDisabled("draft");
  ImGui::SameLine();
  ImGui::TextUnformatted(g.origin);
  if (dirty) {
    ImGui::SameLine();
    ImGui::TextColored(COL_CAUTION, "(modified)");
  }

  /* file / source actions */
  if (ImGui::Button("New (default)")) {
    g.draft = engine_config_default();
    g.baseline = g.draft;
    set_origin("(new)");
    event_log_push(log, sim_time_s, EVENT_INFO, "SPEC",
                   "editor: new spec from the default engine");
  }
  wrap_next(button_width("Copy running"));
  if (ImGui::Button("Copy running")) {
    g.draft = *running;
    g.baseline = *running;
    set_origin(running_name && running_name[0] ? running_name
                                               : "built-in default");
    event_log_push(log, sim_time_s, EVENT_INFO, "SPEC",
                   "editor: copied the running engine");
  }
  wrap_next(button_width("Open..."));
  if (ImGui::Button("Open...")) {
    SDL_ShowOpenFileDialog(on_open_chosen, NULL, window, SPEC_FILTERS, 2, NULL,
                           false);
  }
  wrap_next(button_width("Save as..."));
  if (ImGui::Button("Save as...")) {
    SDL_ShowSaveFileDialog(on_save_chosen, NULL, window, SPEC_FILTERS, 2, NULL);
  }

  /* validation */
  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  const int issues =
      engine_config_check(&g.draft, msgs, ENGINE_CONFIG_MAX_ISSUES);

  ImGui::BeginDisabled(issues > 0);
  if (ImGui::Button("Apply (restart simulation)")) {
    res.apply = true;
    res.config = g.draft;
    snprintf(res.name, sizeof res.name, "%s",
             (dirty || strcmp(g.origin, "(new)") == 0) ? "(unsaved draft)"
                                                       : g.origin);
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(issues > 0
                          ? "fix the validation issues first"
                          : "Cold-start the simulation with this engine.\nSave "
                            "it first if recordings should name the file.");
  }

  if (issues == 0) {
    ImGui::TextDisabled("valid");
  } else {
    const int shown = issues < ENGINE_CONFIG_MAX_ISSUES
                          ? issues
                          : ENGINE_CONFIG_MAX_ISSUES;
    for (int i = 0; i < shown; i++) {
      ImGui::PushTextWrapPos(0.0f);
      ImGui::TextColored(COL_WARNING, "%s", msgs[i]);
      ImGui::PopTextWrapPos();
    }
  }
  ImGui::Separator();

  /* cylinders and firing order */
  ImGui::TextDisabled("LAYOUT");
  int n = g.draft.num_cylinders;
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
  if (ImGui::SliderInt("cylinders", &n, 1, ENGINE_MAX_CYLINDERS)) {
    g.draft.num_cylinders = n;
    engine_default_firing_order(n, g.draft.firing_order);
  }
  const int nc = g.draft.num_cylinders < 1
                     ? 1
                     : (g.draft.num_cylinders > ENGINE_MAX_CYLINDERS
                            ? ENGINE_MAX_CYLINDERS
                            : g.draft.num_cylinders);
  ImGui::TextUnformatted("firing order");
  for (int i = 0; i < nc; i++) {
    ImGui::SameLine();
    ImGui::PushID(i);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 3.2f);
    ImGui::InputInt("##o", &g.draft.firing_order[i], 0, 0);
    ImGui::PopID();
  }
  for (int i = nc; i < ENGINE_MAX_CYLINDERS; i++) {
    g.draft.firing_order[i] = 0; /* slots past the count must stay 0 */
  }
  if (ImGui::Button("Typical order")) {
    engine_default_firing_order(nc, g.draft.firing_order);
  }
  ImGui::SameLine();
  if (ImGui::Button("Sequential order")) {
    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
      g.draft.firing_order[i] = i < nc ? i + 1 : 0;
    }
  }
  ImGui::Separator();

  /* parameters */
  const EngineConfig defaults = engine_config_default();
  EngineConfig defaults_mut = defaults;
  int group = -1;
  if (ImGui::BeginTable("fields", 3)) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 14.0f);
    ImGui::TableSetupColumn("slider", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("reset", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 2.0f);
    for (int i = 0; i < FIELD_COUNT; i++) {
      const Field &f = FIELDS[i];
      if (f.group != group) {
        group = f.group;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("%s", GROUPS[group]);
      }
      double *v = f.get(g.draft);
      const double def = *f.get(defaults_mut);
      const bool has_typ = f.typ_hi > f.typ_lo;
      const bool unusual = has_typ && (*v < f.typ_lo || *v > f.typ_hi);

      ImGui::PushID(i);
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      if (unusual) {
        ImGui::TextColored(COL_CAUTION, "%s", f.label);
      } else {
        ImGui::TextUnformatted(f.label);
      }
      if (ImGui::IsItemHovered()) {
        if (has_typ) {
          ImGui::SetTooltip("%s\ntypical %.4g - %.4g   default %.4g%s", f.key,
                            f.typ_lo, f.typ_hi, def,
                            unusual ? "\noutside the typical range" : "");
        } else {
          ImGui::SetTooltip("%s\ndefault %.4g", f.key, def);
        }
      }
      ImGui::TableSetColumnIndex(1);
      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::SliderScalar("##v", ImGuiDataType_Double, v, &f.lo, &f.hi, f.fmt);
      ImGui::TableSetColumnIndex(2);
      ImGui::BeginDisabled(*v == def);
      if (ImGui::Button("R")) {
        *v = def;
      }
      ImGui::EndDisabled();
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::TextDisabled("Ctrl+click a slider to type a value; amber = outside "
                      "the typical range");

  /* derived figures for the draft */
  ImGui::Separator();
  ImGui::TextDisabled("DERIVED");
  const EngineDerived d = engine_config_derived(&g.draft);
  if (ImGui::BeginTable("derived", 2)) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::GetFontSize() * 14.0f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    derived_row("total displacement", d.total_displacement_l, "L");
    derived_row("per cylinder", d.displacement_per_cyl_l, "L");
    derived_row("clearance volume", d.clearance_cc, "cc/cyl");
    derived_row("firing interval", d.firing_interval_deg, "deg");
    derived_row("bore / stroke", d.bore_stroke_ratio, "");
    derived_row("rod ratio", d.rod_ratio, "");
    derived_row("piston speed @ 3000 rpm", d.piston_speed_3000rpm_ms, "m/s");
    ImGui::EndTable();
  }

  ImGui::End();
  return res;
}
