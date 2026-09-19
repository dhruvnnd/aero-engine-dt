#ifndef UI_EVENT_LOG_PANEL_H
#define UI_EVENT_LOG_PANEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <SDL3/SDL.h>

#include "telemetry/event_log.h"

void event_log_panel_draw(SDL_Renderer *r, float w, float h,
                          const EventLog *log);

#ifdef __cplusplus
}
#endif

#endif /* UI_EVENT_LOG_PANEL_H */
