#ifndef UI_ENGINE_SPEC_PANEL_H
#define UI_ENGINE_SPEC_PANEL_H

#include "model/sync.h"

/* Dockable read-only view of the configuration the simulation is running:
 * the engine spec (layout, dynamics, geometry, combustion), values derived from
 * it, and the thermal / fuel / lubrication / electrical model configs.
 * `spec_name` is the spec file in use, or "" for the built-in default. `open`
 * is cleared when the user closes the window. */
void engine_spec_panel_draw(bool *open, const ModelSync *sync,
                            const char *spec_name);

#endif /* UI_ENGINE_SPEC_PANEL_H */
