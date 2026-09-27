#include "ui/custom_layouts.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "imgui.h"

#define CUSTOM_LAYOUTS_FILE "aero_engine_dt_layouts.txt"
#define LINE_CAP 4096

/* Maps each PanelVisibility bool to a stable on-disk key, so the save file
 * stays readable (and field order in the struct can change freely). */
struct PanelFlagEntry {
  const char *key;
  size_t offset;
};

static const PanelFlagEntry kPanelFlags[] = {
    {"alarms", offsetof(PanelVisibility, alarms)},
    {"sim", offsetof(PanelVisibility, sim)},
    {"instruments", offsetof(PanelVisibility, instruments)},
    {"environment", offsetof(PanelVisibility, environment)},
    {"cylinders", offsetof(PanelVisibility, cylinders)},
    {"trends", offsetof(PanelVisibility, trends)},
    {"cyl_trends", offsetof(PanelVisibility, cyl_trends)},
    {"torque_trace", offsetof(PanelVisibility, torque_trace)},
    {"ecu", offsetof(PanelVisibility, ecu)},
    {"ecu_trends", offsetof(PanelVisibility, ecu_trends)},
    {"ecu_io", offsetof(PanelVisibility, ecu_io)},
    {"ecu_compare", offsetof(PanelVisibility, ecu_compare)},
    {"ecu_faults", offsetof(PanelVisibility, ecu_faults)},
    {"engine_faults", offsetof(PanelVisibility, engine_faults)},
    {"event_log", offsetof(PanelVisibility, event_log)},
    {"gamepad", offsetof(PanelVisibility, gamepad)},
    {"controls", offsetof(PanelVisibility, controls)},
    {"faults", offsetof(PanelVisibility, faults)},
    {"engine_spec", offsetof(PanelVisibility, engine_spec)},
    {"spec_editor", offsetof(PanelVisibility, spec_editor)},
    {"ve_curve", offsetof(PanelVisibility, ve_curve)},
};
#define PANEL_FLAG_COUNT (sizeof kPanelFlags / sizeof kPanelFlags[0])

static bool *flag_at(PanelVisibility *panels, size_t offset) {
  return (bool *)((char *)panels + offset);
}
static const bool *flag_at(const PanelVisibility *panels, size_t offset) {
  return (const bool *)((const char *)panels + offset);
}

static void strip_eol(char *line) {
  char *nl = strpbrk(line, "\r\n");
  if (nl) {
    *nl = '\0';
  }
}

void custom_layouts_load(CustomLayoutSet *set) {
  set->count = 0;
  FILE *f = fopen(CUSTOM_LAYOUTS_FILE, "r");
  if (!f) {
    return;
  }

  char line[LINE_CAP];
  CustomLayout *cur = NULL;
  bool in_ini = false;
  size_t ini_len = 0;

  while (fgets(line, sizeof line, f)) {
    if (!strncmp(line, "==LAYOUT==", 10)) {
      if (set->count >= CUSTOM_LAYOUT_MAX) {
        cur = NULL; /* drop anything past our cap rather than overflow */
        continue;
      }
      cur = &set->items[set->count++];
      memset(cur, 0, sizeof *cur);
      in_ini = false;
      ini_len = 0;
      continue;
    }
    if (!cur) {
      continue;
    }
    if (!in_ini && !strncmp(line, "name=", 5)) {
      strip_eol(line);
      snprintf(cur->name, sizeof cur->name, "%s", line + 5);
    } else if (!in_ini && !strncmp(line, "panels=", 7)) {
      strip_eol(line);
      char *tok = strtok(line + 7, ",");
      while (tok) {
        for (size_t k = 0; k < PANEL_FLAG_COUNT; k++) {
          if (!strcmp(tok, kPanelFlags[k].key)) {
            *flag_at(&cur->panels, kPanelFlags[k].offset) = true;
            break;
          }
        }
        tok = strtok(NULL, ",");
      }
    } else if (!in_ini && !strncmp(line, "==INI==", 7)) {
      in_ini = true;
    } else if (in_ini) {
      size_t len = strlen(line);
      size_t cap = sizeof cur->dock_ini - 1;
      if (ini_len + len < cap) {
        memcpy(cur->dock_ini + ini_len, line, len);
        ini_len += len;
        cur->dock_ini[ini_len] = '\0';
      } /* else: silently truncated; the layout is still mostly usable */
    }
  }
  fclose(f);
}

void custom_layouts_save(const CustomLayoutSet *set) {
  FILE *f = fopen(CUSTOM_LAYOUTS_FILE, "w");
  if (!f) {
    return;
  }
  for (int i = 0; i < set->count; i++) {
    const CustomLayout *it = &set->items[i];
    fprintf(f, "==LAYOUT==\nname=%s\npanels=", it->name);
    bool first = true;
    for (size_t k = 0; k < PANEL_FLAG_COUNT; k++) {
      if (*flag_at(&it->panels, kPanelFlags[k].offset)) {
        fprintf(f, "%s%s", first ? "" : ",", kPanelFlags[k].key);
        first = false;
      }
    }
    fprintf(f, "\n==INI==\n%s", it->dock_ini);
    size_t n = strlen(it->dock_ini);
    if (n && it->dock_ini[n - 1] != '\n') {
      fprintf(f, "\n");
    }
  }
  fclose(f);
}

bool custom_layouts_capture(CustomLayoutSet *set, const char *name,
                            const PanelVisibility *panels) {
  if (!name || !name[0]) {
    return false;
  }

  int idx = -1;
  for (int i = 0; i < set->count; i++) {
    if (!strcmp(set->items[i].name, name)) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    if (set->count < CUSTOM_LAYOUT_MAX) {
      idx = set->count++;
    } else {
      /* full: evict the oldest to make room for the new one */
      memmove(&set->items[0], &set->items[1],
              sizeof(CustomLayout) * (CUSTOM_LAYOUT_MAX - 1));
      idx = CUSTOM_LAYOUT_MAX - 1;
    }
  }

  CustomLayout *it = &set->items[idx];
  memset(it, 0, sizeof *it);
  snprintf(it->name, sizeof it->name, "%s", name);
  it->panels = *panels;

  size_t out_size = 0;
  const char *ini = ImGui::SaveIniSettingsToMemory(&out_size);
  size_t n = out_size < sizeof it->dock_ini - 1 ? out_size
                                                : sizeof it->dock_ini - 1;
  memcpy(it->dock_ini, ini, n);
  it->dock_ini[n] = '\0';

  custom_layouts_save(set);
  return true;
}

void custom_layouts_apply(const CustomLayoutSet *set, int index,
                          PanelVisibility *panels) {
  if (index < 0 || index >= set->count) {
    return;
  }
  const CustomLayout *it = &set->items[index];
  *panels = it->panels;
  ImGui::LoadIniSettingsFromMemory(it->dock_ini);
}

void custom_layouts_remove(CustomLayoutSet *set, int index) {
  if (index < 0 || index >= set->count) {
    return;
  }
  for (int i = index; i < set->count - 1; i++) {
    set->items[i] = set->items[i + 1];
  }
  set->count--;
  custom_layouts_save(set);
}
