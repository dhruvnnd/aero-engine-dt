#include "ui/ui_layout.h"

#include <stddef.h>

static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static float nonneg(float v) { return v < 0.0f ? 0.0f : v; }

UiRect ui_rect_inset(UiRect r, float dx, float dy) {
  return ui_rect_pad(r, dx, dy, dx, dy);
}

UiRect ui_rect_pad(UiRect r, float left, float top, float right, float bottom) {
  UiRect out;
  out.x = r.x + left;
  out.y = r.y + top;
  out.w = nonneg(r.w - left - right);
  out.h = nonneg(r.h - top - bottom);
  return out;
}

int ui_rect_valid(UiRect r) { return r.w > 0.0f && r.h > 0.0f; }

UiRect ui_split_top(UiRect r, float amount, float gap, UiRect *rest) {
  amount = clampf(amount, 0.0f, r.h);
  UiRect strip = {r.x, r.y, r.w, amount};
  if (rest != NULL) {
    float used = amount + gap;
    used = used > r.h ? r.h : used;
    *rest = (UiRect){r.x, r.y + used, r.w, nonneg(r.h - used)};
  }
  return strip;
}

UiRect ui_split_bottom(UiRect r, float amount, float gap, UiRect *rest) {
  amount = clampf(amount, 0.0f, r.h);
  UiRect strip = {r.x, r.y + r.h - amount, r.w, amount};
  if (rest != NULL) {
    float used = amount + gap;
    used = used > r.h ? r.h : used;
    *rest = (UiRect){r.x, r.y, r.w, nonneg(r.h - used)};
  }
  return strip;
}

UiRect ui_split_left(UiRect r, float amount, float gap, UiRect *rest) {
  amount = clampf(amount, 0.0f, r.w);
  UiRect strip = {r.x, r.y, amount, r.h};
  if (rest != NULL) {
    float used = amount + gap;
    used = used > r.w ? r.w : used;
    *rest = (UiRect){r.x + used, r.y, nonneg(r.w - used), r.h};
  }
  return strip;
}

UiRect ui_split_right(UiRect r, float amount, float gap, UiRect *rest) {
  amount = clampf(amount, 0.0f, r.w);
  UiRect strip = {r.x + r.w - amount, r.y, amount, r.h};
  if (rest != NULL) {
    float used = amount + gap;
    used = used > r.w ? r.w : used;
    *rest = (UiRect){r.x, r.y, nonneg(r.w - used), r.h};
  }
  return strip;
}

UiRect ui_split_top_frac(UiRect r, float frac, float gap, UiRect *rest) {
  return ui_split_top(r, r.h * clampf(frac, 0.0f, 1.0f), gap, rest);
}

UiRect ui_split_left_frac(UiRect r, float frac, float gap, UiRect *rest) {
  return ui_split_left(r, r.w * clampf(frac, 0.0f, 1.0f), gap, rest);
}

UiGrid ui_grid(UiRect area, int cols, int rows, float gap) {
  UiGrid g;
  g.area = area;
  g.cols = cols < 1 ? 1 : cols;
  g.rows = rows < 1 ? 1 : rows;
  g.gap = nonneg(gap);
  return g;
}

UiRect ui_grid_cell(UiGrid g, int col, int row) {
  if (col < 0 || col >= g.cols || row < 0 || row >= g.rows) {
    return (UiRect){0, 0, 0, 0};
  }
  float cw = (g.area.w - g.gap * (float)(g.cols - 1)) / (float)g.cols;
  float ch = (g.area.h - g.gap * (float)(g.rows - 1)) / (float)g.rows;
  cw = nonneg(cw);
  ch = nonneg(ch);
  return (UiRect){g.area.x + (float)col * (cw + g.gap),
                  g.area.y + (float)row * (ch + g.gap), cw, ch};
}

UiRect ui_grid_at(UiGrid g, int index) {
  if (index < 0 || index >= g.cols * g.rows) {
    return (UiRect){0, 0, 0, 0};
  }
  return ui_grid_cell(g, index % g.cols, index / g.cols);
}

UiStack ui_stack(UiRect area, float row_gap) {
  UiStack s;
  s.area = area;
  s.cursor_y = area.y;
  s.row_gap = nonneg(row_gap);
  return s;
}

UiRect ui_stack_row(UiStack *s, float height) {
  float bottom = s->area.y + s->area.h;
  float avail = nonneg(bottom - s->cursor_y);
  float h = clampf(height, 0.0f, avail);
  UiRect row = {s->area.x, s->cursor_y, s->area.w, h};
  s->cursor_y += h + s->row_gap;
  return row;
}

UiRect ui_stack_rest(const UiStack *s) {
  float bottom = s->area.y + s->area.h;
  return (UiRect){s->area.x, s->cursor_y, s->area.w,
                  nonneg(bottom - s->cursor_y)};
}
