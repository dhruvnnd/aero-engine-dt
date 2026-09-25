#ifndef UI_SPEC_EDITOR_PANEL_H
#define UI_SPEC_EDITOR_PANEL_H

#include <SDL3/SDL.h>

#include "physics/engine_model.h"
#include "telemetry/event_log.h"

/* What the editor asks the app to do this frame. */
struct SpecEditorResult {
  bool apply;          /* restart the simulation with `config` */
  EngineConfig config; /* valid whenever `apply` is set */
  char name[256];      /* label for it: the saved file, or "(unsaved draft)" */
};

/* Dockable engine-spec editor: start from the default engine, a copy of the
 * running one, or a spec file; edit the layout, dynamics, geometry and
 * combustion parameters with live validation and derived figures; save as a
 * spec file (the same format the CLI tools read) or apply to the running
 * simulation. Open / save use the system file dialogs, which need `window`.
 * Progress and problems go to `log`. `running` / `running_name` seed the draft
 * the first time the panel is drawn. `open` is cleared when the user closes
 * the window. */
SpecEditorResult spec_editor_panel_draw(bool *open, const EngineConfig *running,
                                        const char *running_name,
                                        SDL_Window *window, EventLog *log,
                                        double sim_time_s);

#endif /* UI_SPEC_EDITOR_PANEL_H */
