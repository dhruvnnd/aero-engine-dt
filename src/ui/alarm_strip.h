#ifndef UI_ALARM_STRIP_H
#define UI_ALARM_STRIP_H

#include "telemetry/annunciator.h"

/* Dockable "Alarms" panel: MASTER WARNING / MASTER CAUTION buttons (lit and
 * flashing while unacknowledged, coloured text once acknowledged) followed by
 * the monitored channel names, coloured amber / red when out of limits.
 * `open` is cleared when the user closes the window. Returns true when the
 * user clicked a master button to acknowledge. */
bool alarm_strip_draw(bool *open, const Annunciator *ann);

/* One-line "WARNING n  CAUTION m" text right-aligned in the main menu bar, for
 * when the panel is hidden. Draws nothing when there are no active alarms.
 * Call between BeginMainMenuBar() and EndMainMenuBar(). */
void alarm_menu_indicator(const Annunciator *ann);

#endif /* UI_ALARM_STRIP_H */
