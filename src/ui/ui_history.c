#include "ui/ui_history.h"

#include <stddef.h>

void ui_history_init(UiHistory *h, double *storage, int cap) {
  h->buf = storage;
  h->cap = cap < 0 ? 0 : cap;
  h->count = 0;
  h->head = 0;
}

void ui_history_clear(UiHistory *h) {
  h->count = 0;
  h->head = 0;
}

void ui_history_push(UiHistory *h, double value) {
  if (h->buf == NULL || h->cap == 0) {
    return;
  }
  int tail = (h->head + h->count) % h->cap;
  h->buf[tail] = value;
  if (h->count < h->cap) {
    h->count++;
  } else {
    h->head = (h->head + 1) % h->cap; /* full: overwrite oldest */
  }
}

int ui_history_count(const UiHistory *h) { return h->count; }

double ui_history_at(const UiHistory *h, int i) {
  if (i < 0 || i >= h->count) {
    return 0.0;
  }
  return h->buf[(h->head + i) % h->cap];
}

double ui_history_last(const UiHistory *h) {
  if (h->count == 0) {
    return 0.0;
  }
  return ui_history_at(h, h->count - 1);
}

double ui_history_min(const UiHistory *h) {
  if (h->count == 0) {
    return 0.0;
  }
  double m = ui_history_at(h, 0);
  for (int i = 1; i < h->count; i++) {
    double v = ui_history_at(h, i);
    if (v < m) {
      m = v;
    }
  }
  return m;
}

double ui_history_max(const UiHistory *h) {
  if (h->count == 0) {
    return 0.0;
  }
  double m = ui_history_at(h, 0);
  for (int i = 1; i < h->count; i++) {
    double v = ui_history_at(h, i);
    if (v > m) {
      m = v;
    }
  }
  return m;
}
