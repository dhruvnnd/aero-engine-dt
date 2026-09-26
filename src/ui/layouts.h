#ifndef UI_LAYOUTS_H
#define UI_LAYOUTS_H

#include "imgui.h"

/* Which dockable panels are open. */
struct PanelVisibility {
  bool alarms;
  bool sim;
  bool instruments;
  bool environment;
  bool cylinders;
  bool trends;
  bool cyl_trends;
  bool torque_trace;
  bool ecu;
  bool ecu_trends;
  bool ecu_io;
  bool ecu_compare;
  bool event_log;
  bool gamepad;
  bool controls;
  bool faults;
  bool engine_spec;
  bool spec_editor;
};

enum {
  LAYOUT_OVERVIEW = 0,
  LAYOUT_TRENDS,
  LAYOUT_SYSTEMS,
  LAYOUT_INPUT_LOG,
  LAYOUT_CONFIG,
  LAYOUT_COUNT
};

const char *layout_name(int id);

/* The panels a layout shows (every layout docks all panels, so one opened
 * later lands in a sensible place; this is just the initially open set). */
PanelVisibility layout_visibility(int id);

/* Stable id of the main dockspace. */
ImGuiID layout_dockspace_id();

/* True once the dockspace exists, i.e. a dock layout was restored from the
 * .ini. Call right after NewFrame() on the first frame. */
bool layout_dockspace_exists();

/* Rebuild the dockspace's tree for layout `id`, filling `size`. Call after
 * NewFrame() and BEFORE DockSpaceOverViewport() in the same frame. */
void layout_apply(int id, ImVec2 size);

#endif /* UI_LAYOUTS_H */
