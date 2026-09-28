#include "ui/custom_layouts.h"

#include <direct.h>
#include <dirent.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imgui.h"

#define LINE_CAP 4096

/* Maps each PanelVisibility bool to a stable on-disk key, so the [Panels]
 * section stays readable (and field order in the struct can change freely). */
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
    {"intake", offsetof(PanelVisibility, intake)},
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

/* Windows forbids these in file names; swap them for '_' and trim the
 * trailing dots/spaces it also silently strips. */
static void sanitize_name(const char *in, char *out, size_t cap) {
  size_t j = 0;
  for (size_t i = 0; in[i] && j + 1 < cap; i++) {
    char c = in[i];
    if (strchr("<>:\"/\\|?*", c) || (unsigned char)c < 0x20) {
      c = '_';
    }
    out[j++] = c;
  }
  out[j] = '\0';
  while (j > 0 && (out[j - 1] == ' ' || out[j - 1] == '.')) {
    out[--j] = '\0';
  }
}

static void layout_path(const char *name, char *out, size_t cap) {
  snprintf(out, cap, "%s/%s.ini", CUSTOM_LAYOUT_DIR, name);
}

/* Reads one layouts/<name>.ini: everything up to the "[Panels]" line is the
 * raw ImGui dock/window blob, verbatim; everything after is `key=0`/`key=1`
 * lines for the panel visibility flags. */
static bool load_one(const char *path, CustomLayout *out) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    return false;
  }
  memset(out, 0, sizeof *out);

  char line[LINE_CAP];
  size_t ini_len = 0;
  bool in_panels = false;
  while (fgets(line, sizeof line, f)) {
    if (!in_panels && !strncmp(line, "[Panels]", 8)) {
      in_panels = true;
      continue;
    }
    if (!in_panels) {
      size_t len = strlen(line);
      size_t cap = sizeof out->dock_ini - 1;
      if (ini_len + len < cap) {
        memcpy(out->dock_ini + ini_len, line, len);
        ini_len += len;
        out->dock_ini[ini_len] = '\0';
      } /* else: silently truncated; the layout is still mostly usable */
      continue;
    }
    char *eq = strchr(line, '=');
    if (!eq || atoi(eq + 1) == 0) {
      continue;
    }
    size_t klen = (size_t)(eq - line);
    for (size_t k = 0; k < PANEL_FLAG_COUNT; k++) {
      if (klen == strlen(kPanelFlags[k].key) &&
          !strncmp(line, kPanelFlags[k].key, klen)) {
        *flag_at(&out->panels, kPanelFlags[k].offset) = true;
        break;
      }
    }
  }
  fclose(f);
  return true;
}

static int compare_layout_name(const void *a, const void *b) {
  return strcmp(((const CustomLayout *)a)->name,
                ((const CustomLayout *)b)->name);
}

void custom_layouts_load(CustomLayoutSet *set) {
  set->count = 0;
  DIR *dir = opendir(CUSTOM_LAYOUT_DIR);
  if (!dir) {
    return;
  }

  struct dirent *entry;
  while (set->count < CUSTOM_LAYOUT_MAX && (entry = readdir(dir)) != NULL) {
    size_t len = strlen(entry->d_name);
    if (len <= 4 || strcmp(entry->d_name + len - 4, ".ini") != 0) {
      continue;
    }
    char path[600];
    snprintf(path, sizeof path, "%s/%s", CUSTOM_LAYOUT_DIR, entry->d_name);
    CustomLayout *slot = &set->items[set->count];
    if (!load_one(path, slot)) {
      continue;
    }
    snprintf(slot->name, sizeof slot->name, "%.*s", (int)(len - 4),
             entry->d_name);
    set->count++;
  }
  closedir(dir);

  qsort(set->items, (size_t)set->count, sizeof(CustomLayout),
        compare_layout_name);
}

bool custom_layouts_capture(CustomLayoutSet *set, const char *raw_name,
                            const PanelVisibility *panels) {
  char name[CUSTOM_LAYOUT_NAME_LEN];
  if (!raw_name || !raw_name[0]) {
    return false;
  }
  sanitize_name(raw_name, name, sizeof name);
  if (!name[0]) {
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
      /* full: evict the oldest (by name order) to make room, on disk too */
      char old_path[600];
      layout_path(set->items[0].name, old_path, sizeof old_path);
      remove(old_path);
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
  size_t n =
      out_size < sizeof it->dock_ini - 1 ? out_size : sizeof it->dock_ini - 1;
  memcpy(it->dock_ini, ini, n);
  it->dock_ini[n] = '\0';

  _mkdir(CUSTOM_LAYOUT_DIR); /* ignore failure: already exists is fine too */
  char path[600];
  layout_path(name, path, sizeof path);
  FILE *f = fopen(path, "w");
  if (!f) {
    return false;
  }
  fputs(it->dock_ini, f);
  size_t dlen = strlen(it->dock_ini);
  if (dlen && it->dock_ini[dlen - 1] != '\n') {
    fputc('\n', f);
  }
  fputs("\n[Panels]\n", f);
  for (size_t k = 0; k < PANEL_FLAG_COUNT; k++) {
    fprintf(f, "%s=%d\n", kPanelFlags[k].key,
            *flag_at(&it->panels, kPanelFlags[k].offset) ? 1 : 0);
  }
  fclose(f);

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
  char path[600];
  layout_path(set->items[index].name, path, sizeof path);
  remove(path);
  for (int i = index; i < set->count - 1; i++) {
    set->items[i] = set->items[i + 1];
  }
  set->count--;
}
