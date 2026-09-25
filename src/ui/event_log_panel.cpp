#include "ui/event_log_panel.h"

#include "imgui.h"

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

static ImVec4 level_color(EventLevel lvl) {
  switch (lvl) {
  case EVENT_CAUTION:
    return ImVec4(1.00f, 0.75f, 0.20f, 1.0f);
  case EVENT_WARNING:
    return ImVec4(1.00f, 0.35f, 0.30f, 1.0f);
  case EVENT_INFO:
  default:
    return ImGui::GetStyle().Colors[ImGuiCol_Text];
  }
}

void event_log_panel_draw(bool *open, const EventLog *log) {
  ImGui::SetNextWindowSize(ImVec2(620.0f, 400.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Event Log", open)) {
    ImGui::End();
    return;
  }

  const int n = event_log_count(log);
  ImGui::TextDisabled("%d logged", n);
  ImGui::Separator();

  if (n == 0) {
    ImGui::TextDisabled("-- no events logged yet --");
    ImGui::End();
    return;
  }

  const float fs = ImGui::GetFontSize();
  const ImGuiTableFlags flags =
      ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
      ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV;
  if (ImGui::BeginTable("events", 4, flags)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed,
                            fs * 6.0f);
    ImGui::TableSetupColumn("Level", ImGuiTableColumnFlags_WidthFixed,
                            fs * 4.5f);
    ImGui::TableSetupColumn("Category", ImGuiTableColumnFlags_WidthFixed,
                            fs * 7.0f);
    ImGui::TableSetupColumn("Message", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

    ImGuiListClipper clipper;
    clipper.Begin(n);
    while (clipper.Step()) {
      for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++) {
        const EventRecord *rec = event_log_at(log, n - 1 - row);
        if (!rec) {
          continue;
        }
        const bool colored = rec->level != EVENT_INFO;
        const ImVec4 col = level_color(rec->level);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("T+%06.1f", rec->sim_time_s);
        ImGui::TableSetColumnIndex(1);
        if (colored) {
          ImGui::PushStyleColor(ImGuiCol_Text, col);
        }
        ImGui::TextUnformatted(level_token(rec->level));
        ImGui::TableSetColumnIndex(2);
        ImGui::TextDisabled("%s", rec->category);
        ImGui::TableSetColumnIndex(3);
        ImGui::TextUnformatted(rec->message);
        if (colored) {
          ImGui::PopStyleColor();
        }
      }
    }
    ImGui::EndTable();
  }

  ImGui::End();
}
