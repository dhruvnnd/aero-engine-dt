#include "ui/layouts.h"

#include "imgui_internal.h"
#include "ui/panel_names.h"

const char *layout_name(int id) {
  switch (id) {
  case LAYOUT_OVERVIEW:
    return "Overview";
  case LAYOUT_TRENDS:
    return "Trends";
  case LAYOUT_SYSTEMS:
    return "Systems";
  case LAYOUT_INPUT_LOG:
    return "Input & Log";
  case LAYOUT_CONFIG:
    return "Configuration";
  default:
    return "?";
  }
}

PanelVisibility layout_visibility(int id) {
  PanelVisibility v = {};
  switch (id) {
  case LAYOUT_TRENDS:
    v.sim = v.instruments = v.environment = v.cylinders = v.trends =
        v.event_log = true;
    break;
  case LAYOUT_SYSTEMS:
    v.sim = v.instruments = v.environment = v.cylinders = v.event_log = true;
    break;
  case LAYOUT_INPUT_LOG:
    v.sim = v.instruments = v.event_log = v.gamepad = true;
    break;
  case LAYOUT_CONFIG:
    v.sim = v.event_log = v.engine_spec = v.spec_editor = true;
    break;
  case LAYOUT_OVERVIEW:
  default:
    v.sim = v.instruments = v.environment = v.cylinders = v.trends =
        v.event_log = true;
    break;
  }
  v.alarms = true; /* thin bar along the bottom; closable, movable */
  v.controls = true; /* tabs beside Sim in every layout */
  v.faults = true;
  v.cyl_trends = true; /* a tab beside Trends / Cylinders */
  v.torque_trace = true;
  v.ecu = true;
  v.ecu_trends = true;
  v.ecu_io = true;
  v.ecu_compare = true;
  v.ecu_faults = true;
  return v;
}

ImGuiID layout_dockspace_id() { return ImHashStr("AeroDockspace"); }

bool layout_dockspace_exists() {
  return ImGui::DockBuilderGetNode(layout_dockspace_id()) != NULL;
}

/* Carves `ratio` of `node` off toward `dir` and returns the carved node;
 * `node` is updated to the remainder. */
static ImGuiID carve(ImGuiID &node, ImGuiDir dir, float ratio) {
  ImGuiID carved = 0;
  ImGuiID rest = 0;
  ImGui::DockBuilderSplitNode(node, dir, ratio, &carved, &rest);
  node = rest;
  return carved;
}

static void dock(ImGuiID node, const char *const *names, int count) {
  for (int i = 0; i < count; i++) {
    ImGui::DockBuilderDockWindow(names[i], node);
  }
}

#define DOCK(node, ...)                                                        \
  do {                                                                         \
    const char *const _n[] = {__VA_ARGS__};                                    \
    dock((node), _n, (int)(sizeof _n / sizeof _n[0]));                         \
  } while (0)

void layout_apply(int id, ImVec2 size) {
  const ImGuiID root = layout_dockspace_id();
  ImGui::DockBuilderRemoveNode(root);
  ImGui::DockBuilderAddNode(root, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(root, size);

  ImGuiID rest = root;
  const float strip_px = ImGui::GetFrameHeight() * 2.7f;
  const ImGuiID alarms = carve(rest, ImGuiDir_Down, strip_px / size.y);
  DOCK(alarms, PANEL_ALARMS);

  switch (id) {
  case LAYOUT_TRENDS: {
    /* big trends on the left; instruments over a tab group on the right */
    ImGuiID right = carve(rest, ImGuiDir_Right, 0.32f);
    ImGuiID right_bottom = carve(right, ImGuiDir_Down, 0.45f);
    DOCK(rest, PANEL_TRENDS, PANEL_CYL_TRENDS, PANEL_TORQUE_TRACE,
         PANEL_ECU_TRENDS, PANEL_ECU_COMPARE);
    DOCK(right, PANEL_INSTRUMENTS);
    DOCK(right_bottom, PANEL_CYLINDERS, PANEL_ENVIRONMENT, PANEL_SIM,
         PANEL_CONTROLS, PANEL_ECU, PANEL_ECU_IO, PANEL_ECU_FAULTS, PANEL_FAULTS, PANEL_EVENT_LOG, PANEL_GAMEPAD,
         PANEL_ENGINE_SPEC, PANEL_SPEC_EDITOR);
    break;
  }
  case LAYOUT_SYSTEMS: {
    /* no plots: instruments beside cylinders / environment / sim + log */
    ImGuiID left = carve(rest, ImGuiDir_Left, 0.50f);
    ImGuiID cylinders = carve(rest, ImGuiDir_Up, 0.45f);
    ImGuiID environment = carve(rest, ImGuiDir_Up, 0.35f);
    DOCK(left, PANEL_INSTRUMENTS);
    DOCK(cylinders, PANEL_CYLINDERS, PANEL_CYL_TRENDS, PANEL_TORQUE_TRACE,
         PANEL_ECU_TRENDS, PANEL_ECU_COMPARE);
    DOCK(environment, PANEL_ENVIRONMENT);
    DOCK(rest, PANEL_SIM, PANEL_CONTROLS, PANEL_ECU, PANEL_ECU_IO, PANEL_ECU_FAULTS, PANEL_FAULTS, PANEL_EVENT_LOG,
         PANEL_GAMEPAD,
         PANEL_TRENDS, PANEL_ENGINE_SPEC, PANEL_SPEC_EDITOR);
    break;
  }
  case LAYOUT_INPUT_LOG: {
    /* event log and gamepad front and centre */
    ImGuiID right = carve(rest, ImGuiDir_Right, 0.40f);
    ImGuiID right_bottom = carve(right, ImGuiDir_Down, 0.35f);
    ImGuiID left_bottom = carve(rest, ImGuiDir_Down, 0.40f);
    DOCK(rest, PANEL_EVENT_LOG);
    DOCK(left_bottom, PANEL_INSTRUMENTS, PANEL_CYLINDERS, PANEL_TRENDS,
         PANEL_CYL_TRENDS, PANEL_TORQUE_TRACE, PANEL_ECU_TRENDS, PANEL_ECU_COMPARE);
    DOCK(right, PANEL_GAMEPAD);
    DOCK(right_bottom, PANEL_SIM, PANEL_CONTROLS, PANEL_ECU, PANEL_ECU_IO, PANEL_ECU_FAULTS, PANEL_FAULTS,
         PANEL_ENVIRONMENT, PANEL_ENGINE_SPEC, PANEL_SPEC_EDITOR);
    break;
  }
  case LAYOUT_CONFIG: {
    /* engine spec beside its editor; log and sim controls underneath, every
     * other panel tabbed with them (closed until opened from View) */
    ImGuiID left = carve(rest, ImGuiDir_Left, 0.38f);
    ImGuiID bottom = carve(rest, ImGuiDir_Down, 0.24f);
    DOCK(left, PANEL_ENGINE_SPEC);
    DOCK(rest, PANEL_SPEC_EDITOR);
    DOCK(bottom, PANEL_EVENT_LOG, PANEL_SIM, PANEL_CONTROLS, PANEL_FAULTS,
         PANEL_INSTRUMENTS, PANEL_CYLINDERS, PANEL_ENVIRONMENT, PANEL_TRENDS,
         PANEL_CYL_TRENDS, PANEL_TORQUE_TRACE, PANEL_ECU, PANEL_ECU_IO, PANEL_ECU_FAULTS, PANEL_ECU_TRENDS, PANEL_ECU_COMPARE, PANEL_GAMEPAD);
    break;
  }
  case LAYOUT_OVERVIEW:
  default: {
    /* instruments + tab group on the left, trends over the log on the right */
    ImGuiID left = carve(rest, ImGuiDir_Left, 0.40f);
    ImGuiID left_bottom = carve(left, ImGuiDir_Down, 0.38f);
    ImGuiID right_bottom = carve(rest, ImGuiDir_Down, 0.28f);
    DOCK(left, PANEL_INSTRUMENTS);
    DOCK(left_bottom, PANEL_CYLINDERS, PANEL_ENVIRONMENT, PANEL_SIM,
         PANEL_CONTROLS, PANEL_ECU, PANEL_ECU_IO, PANEL_ECU_FAULTS, PANEL_FAULTS, PANEL_ENGINE_SPEC,
         PANEL_SPEC_EDITOR);
    DOCK(rest, PANEL_TRENDS, PANEL_CYL_TRENDS, PANEL_TORQUE_TRACE,
         PANEL_ECU_TRENDS, PANEL_ECU_COMPARE);
    DOCK(right_bottom, PANEL_EVENT_LOG, PANEL_GAMEPAD);
    break;
  }
  }

  ImGui::DockBuilderFinish(root);
}
