#include "ui/alarm_strip.h"

#include <math.h>
#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 CAUTION(1.00f, 0.75f, 0.20f, 1.0f);
static const ImVec4 CAUTION_ON_TEXT(0.10f, 0.07f, 0.00f, 1.0f);
static const ImVec4 WARNING(0.90f, 0.22f, 0.20f, 1.0f);
static const ImVec4 WARNING_ON_TEXT(1.00f, 1.00f, 1.00f, 1.0f);

static bool blink_phase() { return fmod(ImGui::GetTime(), 0.7) < 0.35; }

/* A plain button: filled while flashing (unacknowledged), coloured text while
 * active but acknowledged, dim when nothing is active. */
static bool master_button(const char *label, int active, int unacked,
                          const ImVec4 &color, const ImVec4 &on_text) {
  int pushed = 0;
  if (unacked > 0 && blink_phase()) {
    ImGui::PushStyleColor(ImGuiCol_Button, color);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);
    ImGui::PushStyleColor(ImGuiCol_Text, on_text);
    pushed = 4;
  } else if (active > 0) {
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    pushed = 1;
  } else {
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
    pushed = 1;
  }
  const bool clicked = ImGui::Button(label);
  ImGui::PopStyleColor(pushed);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%d active, %d unacknowledged\nClick or press SPACE to "
                      "acknowledge",
                      active, unacked);
  }
  return clicked;
}

bool alarm_strip_draw(bool *open, const Annunciator *ann) {
  if (!ImGui::Begin(PANEL_ALARMS, open,
                    ImGuiWindowFlags_NoScrollbar |
                        ImGuiWindowFlags_NoScrollWithMouse)) {
    ImGui::End();
    return false;
  }

  const int warn_active = annunciator_active(ann, CHANNEL_ALERT);
  const int warn_unacked = annunciator_unacked(ann, CHANNEL_ALERT);
  const int caut_active = annunciator_active(ann, CHANNEL_WARN);
  const int caut_unacked = annunciator_unacked(ann, CHANNEL_WARN);

  bool ack_clicked = false;
  if (master_button("MASTER WARNING", warn_active, warn_unacked, WARNING,
                    WARNING_ON_TEXT)) {
    ack_clicked = true;
  }
  ImGui::SameLine();
  if (master_button("MASTER CAUTION", caut_active, caut_unacked, CAUTION,
                    CAUTION_ON_TEXT)) {
    ack_clicked = true;
  }

  const float gap = ImGui::GetFontSize() * 0.9f;
  for (int i = 0; i < MON_CHANNELS; i++) {
    const MonitorChannel ch = (MonitorChannel)i;
    const ChannelStatus st = annunciator_status(ann, ch);
    ImGui::SameLine(0.0f, i == 0 ? gap * 2.0f : gap);
    if (st == CHANNEL_ALERT) {
      ImGui::TextColored(WARNING, "%s", monitor_channel_name(ch));
    } else if (st == CHANNEL_WARN) {
      ImGui::TextColored(CAUTION, "%s", monitor_channel_name(ch));
    } else {
      ImGui::TextDisabled("%s", monitor_channel_name(ch));
    }
  }

  ImGui::SameLine(0.0f, gap * 2.0f);
  if (warn_unacked || caut_unacked) {
    ImGui::TextUnformatted("SPACE = acknowledge");
  } else if (warn_active || caut_active) {
    ImGui::TextDisabled("acknowledged");
  } else {
    ImGui::TextDisabled("no active alarms");
  }

  ImGui::End();
  return ack_clicked;
}

void alarm_menu_indicator(const Annunciator *ann) {
  const int warn = annunciator_active(ann, CHANNEL_ALERT);
  const int caut = annunciator_active(ann, CHANNEL_WARN);
  if (warn == 0 && caut == 0) {
    return;
  }
  const bool unacked = annunciator_unacked(ann, CHANNEL_ALERT) +
                           annunciator_unacked(ann, CHANNEL_WARN) >
                       0;

  char buf[64];
  if (warn > 0 && caut > 0) {
    snprintf(buf, sizeof buf, "WARNING %d  CAUTION %d", warn, caut);
  } else if (warn > 0) {
    snprintf(buf, sizeof buf, "WARNING %d", warn);
  } else {
    snprintf(buf, sizeof buf, "CAUTION %d", caut);
  }

  const float x = ImGui::GetWindowWidth() - ImGui::CalcTextSize(buf).x -
                  ImGui::GetStyle().WindowPadding.x * 2.0f;
  ImGui::SetCursorPosX(x);
  ImVec4 col = warn > 0 ? WARNING : CAUTION;
  if (unacked && !blink_phase()) {
    col.w = 0.35f;
  }
  ImGui::TextColored(col, "%s", buf);
}
