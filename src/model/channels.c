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
};

const ModelChannel *model_channels(int *count) {
  *count = (int)(sizeof(CHANNELS) / sizeof(CHANNELS[0]));
  return CHANNELS;
}
