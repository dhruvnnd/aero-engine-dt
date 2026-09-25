#ifndef UI_EVENT_LOG_PANEL_H
#define UI_EVENT_LOG_PANEL_H

#include "telemetry/event_log.h"

/* Draws the event log as a dockable ImGui window, newest event first.
 *  `open` is cleared when the user closes the window. Call between
 * NewFrame/Render. */
void event_log_panel_draw(bool *open, const EventLog *log);

#endif /* UI_EVENT_LOG_PANEL_H */
