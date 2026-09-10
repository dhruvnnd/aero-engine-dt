#include "model/channels.h"

static double ch_rpm(const ModelState *s, int i) {
  (void)i;
  return s->rpm;
}
static double ch_map_kpa(const ModelState *s, int i) {
  (void)i;
  return s->engine.map_kpa;
}
static double ch_torque_nm(const ModelState *s, int i) {
  (void)i;
  return s->torque_nm;
}
static double ch_cht_c(const ModelState *s, int i) {
  (void)i;
  return s->thermal.cht_c;
}
static double ch_egt_c(const ModelState *s, int i) {
  (void)i;
  return s->thermal.egt_c;
}
static double ch_oil_c(const ModelState *s, int i) {
  (void)i;
  return s->thermal.oil_temp_c;
}
static double ch_cyl_cht(const ModelState *s, int i) { return s->cyl[i].cht_c; }
static double ch_cyl_egt(const ModelState *s, int i) { return s->cyl[i].egt_c; }
static double ch_cyl_lambda(const ModelState *s, int i) {
  return s->cyl[i].lambda;
}
static double ch_air_gps(const ModelState *s, int i) {
  (void)i;
  return s->fuel.air_flow_gps;
}
static double ch_fuel_kgph(const ModelState *s, int i) {
  (void)i;
  return s->fuel.fuel_flow_kgph;
}
static double ch_fuel_press(const ModelState *s, int i) {
  (void)i;
  return s->fuel.fuel_press_kpa;
}
static double ch_oil_press(const ModelState *s, int i) {
  (void)i;
  return s->lube.oil_press_kpa;
}

static const ModelChannel CHANNELS[] = {
    {"rpm", "rpm", 2, -1, ch_rpm},
    {"map_kpa", "kPa", 3, -1, ch_map_kpa},
    {"torque_nm", "N*m", 3, -1, ch_torque_nm},
    {"cht_c", "degC", 3, -1, ch_cht_c},
    {"egt_c", "degC", 3, -1, ch_egt_c},
    {"oil_c", "degC", 3, -1, ch_oil_c},
    {"cht_c_1", "degC", 3, 0, ch_cyl_cht},
    {"cht_c_2", "degC", 3, 1, ch_cyl_cht},
    {"cht_c_3", "degC", 3, 2, ch_cyl_cht},
    {"cht_c_4", "degC", 3, 3, ch_cyl_cht},
    {"egt_c_1", "degC", 3, 0, ch_cyl_egt},
    {"egt_c_2", "degC", 3, 1, ch_cyl_egt},
    {"egt_c_3", "degC", 3, 2, ch_cyl_egt},
    {"egt_c_4", "degC", 3, 3, ch_cyl_egt},
    {"air_gps", "g/s", 3, -1, ch_air_gps},
    {"fuel_kgph", "kg/h", 4, -1, ch_fuel_kgph},
    {"fuel_press_kpa", "kPa", 2, -1, ch_fuel_press},
    {"lambda_1", "-", 4, 0, ch_cyl_lambda},
    {"lambda_2", "-", 4, 1, ch_cyl_lambda},
    {"lambda_3", "-", 4, 2, ch_cyl_lambda},
    {"lambda_4", "-", 4, 3, ch_cyl_lambda},
    {"oil_press_kpa", "kPa", 2, -1, ch_oil_press},
};

const ModelChannel *model_channels(int *count) {
  *count = (int)(sizeof(CHANNELS) / sizeof(CHANNELS[0]));
  return CHANNELS;
}
