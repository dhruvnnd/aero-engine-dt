#include "util/history.h"

#include <stddef.h>

void history_init(History *h, double *storage, int cap) {
  h->buf = storage;
  h->cap = cap < 0 ? 0 : cap;
  h->count = 0;
  h->head = 0;
}

void history_clear(History *h) {
  h->count = 0;
  h->head = 0;
}

void history_push(History *h, double value) {
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

int history_count(const History *h) { return h->count; }

double history_at(const History *h, int i) {
  if (i < 0 || i >= h->count) {
    return 0.0;
  }
  return h->buf[(h->head + i) % h->cap];
}

double history_last(const History *h) {
  if (h->count == 0) {
    return 0.0;
  }
  return history_at(h, h->count - 1);
}

double history_min(const History *h) {
  if (h->count == 0) {
    return 0.0;
  }
  double m = history_at(h, 0);
  for (int i = 1; i < h->count; i++) {
    double v = history_at(h, i);
    if (v < m) {
      m = v;
    }
  }
  return m;
}

double history_max(const History *h) {
  if (h->count == 0) {
    return 0.0;
  }
  double m = history_at(h, 0);
  for (int i = 1; i < h->count; i++) {
    double v = history_at(h, i);
    if (v > m) {
      m = v;
    }
  }
  return m;
}
