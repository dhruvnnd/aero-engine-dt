#ifndef UI_PROFILER_PANEL_H
#define UI_PROFILER_PANEL_H

#include "util/profiler.h"

struct ProfilerPanelInfo {
  int first_panel_stage;
  int physics_stage;
  int shadow_stage;
  int ui_stage;
  int render_stage;
  int present_stage;

  int steps_counter;        /* physics steps this frame */
  int substeps_counter;     /* engine crank sub-steps this frame */
  int shadow_steps_counter; /* shadow-model steps this frame */
  int sim_s_counter;        /* simulated seconds advanced this frame */

  double target_frame_ms; /* display refresh period; 0 if unknown */
  double sim_speed;       /* requested sim speed multiplier */
  bool sim_paused;
  bool shadow_active;
};

void profiler_panel_draw(bool *open, Profiler *p,
                         const ProfilerPanelInfo &info);

#endif /* UI_PROFILER_PANEL_H */
