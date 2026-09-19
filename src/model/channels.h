#ifndef MODEL_CHANNELS_H
#define MODEL_CHANNELS_H

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* MODEL_CHANNELS_H */
