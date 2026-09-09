#ifndef MODEL_CHANNELS_H
#define MODEL_CHANNELS_H

#include "model/state.h"

typedef struct {
  const char *name;
  const char *unit;
  int precision;
  int index;
  double (*get)(const ModelState *s, int index);
} ModelChannel;

/* Returns the channel table and writes its length to *count. The pointer is
 * valid for the life of the process. */
const ModelChannel *model_channels(int *count);

#endif /* MODEL_CHANNELS_H */
