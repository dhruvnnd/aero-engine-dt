#include "ui/event_log_panel.h"

#include "ui/ui_draw.h"
#include "ui/ui_layout.h"
#include "ui/ui_theme.h"

/* Fixed-width bracketed token, matching the "[ OK ]"/"[WARN]"/"[ALRM]" style
 * of ui_widgets' status tokens but keyed off EventLevel instead of UiStatus. */
static const char *level_token(EventLevel lvl) {
  switch (lvl) {
    case EVENT_CAUTION:
      return "[CTN ]";
    case EVENT_WARNING:
      return "[WARN]";
    case EVENT_INFO:
    default:
      return "[INFO]";
  }
}

static SDL_Color level_color(const UiTheme *th, EventLevel lvl) {
  switch (lvl) {
    case EVENT_CAUTION:
      return th->warn;
    case EVENT_WARNING:
      return th->alert;
    case EVENT_INFO:
    default:
      return th->text;
  }
}

void event_log_panel_draw(SDL_Renderer *r, float w, float h,
                          const EventLog *log) {
  const UiTheme th = ui_theme_default();

  ui_fill(r, (UiRect){0.0f, 0.0f, w, h}, th.bg);
  UiRect screen = ui_rect_inset((UiRect){0.0f, 0.0f, w, h}, 8.0f, 8.0f);

  UiRect body;
  UiRect header = ui_split_top(screen, 14.0f, 6.0f, &body);
  ui_text(r, header.x, header.y, th.text_bright, "event log");
  ui_text_right(r, header.x + header.w, header.y, th.text_dim,
                "%d logged", event_log_count(log));
  ui_hline(r, screen.x, header.y + 12.0f, screen.w, th.frame);

  UiRect content;
  UiRect footer = ui_split_bottom(body, 10.0f, 6.0f, &content);
  ui_text(r, footer.x, footer.y, th.text_dim,
          "L  hide / show      close window to dismiss      newest first");

  const int n = event_log_count(log);
  if (n == 0) {
    ui_text(r, content.x, content.y, th.text_dim,
            "-- no events logged yet --");
    return;
  }

  /* Column layout: time | severity token | category | message. */
  const float c1 = content.x;
  const float c2 = c1 + 10.0f * UI_GLYPH_W;
  const float c3 = c2 + 7.0f * UI_GLYPH_W;
  const float c4 = c3 + 11.0f * UI_GLYPH_W;

  const int max_rows = (int)(content.h / th.row_h);
  const int shown = n < max_rows ? n : max_rows;

  float ry = content.y;
  for (int row = 0; row < shown; row++) {
    /* newest first: walk backward from the most recent record */
    const EventRecord *rec = event_log_at(log, n - 1 - row);
    if (!rec) {
      continue;
    }
    SDL_Color col = level_color(&th, rec->level);
    ui_text(r, c1, ry, th.text_dim, "T+%06.1f", rec->sim_time_s);
    ui_text(r, c2, ry, col, "%s", level_token(rec->level));
    ui_text(r, c3, ry, th.text_dim, "%-9s", rec->category);
    ui_text(r, c4, ry, col, "%s", rec->message);
    ry += th.row_h;
  }
}
