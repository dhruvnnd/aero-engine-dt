#ifndef UI_CUSTOM_LAYOUTS_H
#define UI_CUSTOM_LAYOUTS_H

#include "ui/layouts.h"

#define CUSTOM_LAYOUT_MAX 12
#define CUSTOM_LAYOUT_NAME_LEN 48
#define CUSTOM_LAYOUT_INI_CAP 16384
#define CUSTOM_LAYOUT_DIR "layouts"

typedef struct {
  char name[CUSTOM_LAYOUT_NAME_LEN];
  PanelVisibility panels;
  char dock_ini[CUSTOM_LAYOUT_INI_CAP]; /* raw ImGui window/dock settings, as
                                         * from SaveIniSettingsToMemory() */
} CustomLayout;

typedef struct {
  CustomLayout items[CUSTOM_LAYOUT_MAX];
  int count;
} CustomLayoutSet;

/* Scans the layouts folder's .ini files into `set` (cleared first). Leaves
 * `set` empty if the folder doesn't exist yet or nothing in it is readable. */
void custom_layouts_load(CustomLayoutSet *set);

/* Captures the current ImGui dock tree/window layout and `panels` under
 * `name`, writing layouts/<name>.ini. Replaces the existing layout of the same
 * name if there is one; otherwise appends, evicting the oldest entry (and its
 * file) once `set` is full. Does nothing and returns false if `name` is empty.
 */
bool custom_layouts_capture(CustomLayoutSet *set, const char *name,
                            const PanelVisibility *panels);

/* Restores `set->items[index]`: writes its saved panel visibility into
 * `panels` and loads its dock tree into ImGui. */
void custom_layouts_apply(const CustomLayoutSet *set, int index,
                          PanelVisibility *panels);

/* Deletes items[index]'s file and drops it from `set`. */
void custom_layouts_remove(CustomLayoutSet *set, int index);

#endif /* UI_CUSTOM_LAYOUTS_H */
