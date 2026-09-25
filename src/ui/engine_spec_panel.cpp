#include "ui/engine_spec_panel.h"

#include <stdio.h>

#include "imgui.h"
#include "ui/panel_names.h"

static const ImVec4 COL_WARNING(1.00f, 0.35f, 0.30f, 1.0f);

static bool begin_table(const char *id) {
  if (!ImGui::BeginTable(id, 3, ImGuiTableFlags_RowBg)) {
    return false;
  }
  ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 15.0f);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
  ImGui::TableSetupColumn("unit", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 6.0f);
  return true;
}

static void row_text(const char *label, const char *value, const char *unit) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);
  ImGui::TableSetColumnIndex(1);
  const float w = ImGui::CalcTextSize(value).x;
  const float avail = ImGui::GetContentRegionAvail().x;
  if (avail > w) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + avail - w);
  }
  ImGui::TextUnformatted(value);
  ImGui::TableSetColumnIndex(2);
  ImGui::TextDisabled("%s", unit);
}

static void row(const char *label, double v, const char *unit) {
  char buf[48];
  snprintf(buf, sizeof buf, "%.6g", v);
  row_text(label, buf, unit);
}

static void row_int(const char *label, int v, const char *unit) {
  char buf[16];
  snprintf(buf, sizeof buf, "%d", v);
  row_text(label, buf, unit);
}

void engine_spec_panel_draw(bool *open, const ModelSync *sync,
                            const char *spec_name) {
  ImGui::SetNextWindowSize(ImVec2(460.0f, 620.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ENGINE_SPEC, open)) {
    ImGui::End();
    return;
  }

  const EngineConfig &e = sync->engine_config;

  ImGui::TextDisabled("source");
  ImGui::SameLine();
  ImGui::TextUnformatted(spec_name && spec_name[0] ? spec_name
                                                   : "built-in default");

  char msgs[ENGINE_CONFIG_MAX_ISSUES][ENGINE_CONFIG_ISSUE_LEN];
  const int issues = engine_config_check(&e, msgs, ENGINE_CONFIG_MAX_ISSUES);
  if (issues == 0) {
    ImGui::TextDisabled("valid");
  } else {
    const int shown = issues < ENGINE_CONFIG_MAX_ISSUES
                          ? issues
                          : ENGINE_CONFIG_MAX_ISSUES;
    for (int i = 0; i < shown; i++) {
      ImGui::TextColored(COL_WARNING, "%s", msgs[i]);
    }
  }
  ImGui::Separator();

  const EngineDerived d = engine_config_derived(&e);

  if (ImGui::CollapsingHeader("Layout", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_table("layout")) {
    row_int("cylinders", e.num_cylinders, "");
    char order[64] = "";
    int len = 0;
    for (int i = 0; i < e.num_cylinders && i < ENGINE_MAX_CYLINDERS &&
                    len < (int)sizeof order - 4;
         i++) {
      len += snprintf(order + len, sizeof order - (size_t)len, "%s%d",
                      i > 0 ? "-" : "", e.firing_order[i]);
    }
    row_text("firing order", order, "");
    row("firing interval", d.firing_interval_deg, "deg");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Dynamics", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_table("dynamics")) {
    row("crank inertia", e.inertia_kg_m2, "kg*m^2");
    row("MAP time constant", e.map_tau_s, "s");
    row("friction", e.friction_coeff_nm_per_rad_s, "N*m/(rad/s)");
    row("starter torque", e.starter_torque_nm, "N*m");
    row("starter catch speed", e.starter_catch_rpm, "rpm");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Geometry", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_table("geometry")) {
    row("bore", e.geom.bore_m * 1000.0, "mm");
    row("stroke", e.geom.stroke_m * 1000.0, "mm");
    row("conrod length", e.geom.conrod_len_m * 1000.0, "mm");
    row("compression ratio", e.geom.compression_ratio, ":1");
    row("exhaust valve opens", e.geom.evo_deg, "deg");
    row("intake valve closes", e.geom.ivc_deg, "deg");
    row("reciprocating mass", e.geom.m_recip_kg, "kg/cyl");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Combustion") && begin_table("combustion")) {
    row("Wiebe a", e.geom.wiebe_a, "");
    row("Wiebe m", e.geom.wiebe_m, "");
    row("burn duration", e.geom.delta_theta_burn_deg, "deg");
    row("spark base advance", e.geom.spark_base_btdc_deg, "deg BTDC");
    row("spark rpm gain", e.geom.spark_rpm_gain_deg_per_1000rpm, "deg/1000rpm");
    row("spark MAP retard", e.geom.spark_map_retard_deg_per_kpa, "deg/kPa");
    row("combustion efficiency", e.geom.combustion_efficiency, "");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Derived", ImGuiTreeNodeFlags_DefaultOpen) &&
      begin_table("derived")) {
    row("displacement per cyl", d.displacement_per_cyl_l, "L");
    row("total displacement", d.total_displacement_l, "L");
    row("clearance volume", d.clearance_cc, "cc/cyl");
    row("bore / stroke", d.bore_stroke_ratio, "");
    row("rod ratio", d.rod_ratio, "");
    row("piston speed @ 3000 rpm", d.piston_speed_3000rpm_ms, "m/s");
    row("fuel model displacement", sync->fuel_config.displacement_l, "L");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Thermal") && begin_table("thermal")) {
    const ThermalConfig &t = sync->thermal_config;
    row("CHT time constant", t.cht_tau_s, "s");
    row("EGT time constant", t.egt_tau_s, "s");
    row("oil time constant", t.oil_tau_s, "s");
    row("CHT rise at rated load", t.cht_rise_rated_c, "degC");
    row("EGT rise at rated load", t.egt_rise_rated_c, "degC");
    row("oil gain", t.oil_gain_c_per_w, "degC/W");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Fuel") && begin_table("fuel")) {
    const FuelConfig &f = sync->fuel_config;
    row("stoichiometric AFR", f.afr_stoich, "");
    row("target lambda", f.lambda_target, "");
    row("volumetric efficiency", f.vol_eff, "");
    row("displacement", f.displacement_l, "L");
    row("pump pressure", f.pump_press_kpa, "kPa");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Lubrication") && begin_table("lube")) {
    const LubeConfig &l = sync->lube_config;
    row("relief valve", l.relief_valve_kpa, "kPa");
    row("relief band", l.relief_band_kpa, "kPa");
    row("pump gain", l.k_pump_kpa_per_rpm, "kPa/rpm");
    row("viscosity ref temp", l.visc_ref_temp_c, "degC");
    row("viscosity falloff", l.visc_falloff_per_c, "/degC");
    row("bearing wear", l.bearing_wear, "");
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Electrical") && begin_table("elec")) {
    const ElecConfig &c = sync->elec_config;
    row("bus voltage", c.bus_nominal_v, "V");
    row("alternator rated", c.alt_rated_a, "A");
    row("alternator cut-in", c.alt_cutin_rpm, "rpm");
    row("alternator full output", c.alt_full_output_rpm, "rpm");
    row("alternator health", c.alt_health, "");
    row("base load", c.load_base_a, "A");
    row("battery capacity", c.batt_capacity_ah, "Ah");
    row("battery open voltage", c.batt_open_v, "V");
    row("battery internal R", c.batt_internal_r_ohm, "ohm");
    row("starter current", c.starter_current_a, "A");
    ImGui::EndTable();
  }

  ImGui::End();
}
