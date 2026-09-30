#include "telemetry/pv_diagram.h"

#include <math.h>

#include "math/units.h"
#include "physics/crank_thermo.h"

/* Compression-fit window, cylinder-local degrees: from a little after IVC (so
 * the valve's pressure relaxation has died out) to before any spark advance
 * the model can ask for has started to lift the pressure. */
#define FIT_AFTER_IVC_DEG 10.0
#define FIT_BEFORE_TDC_DEG 45.0
#define FIT_MIN_POINTS 4

static double effective_cr(const EngineConfig *cfg, const CylinderConfig *cc) {
  const double trim = cc ? cc->compression_trim : 1.0;
  return cfg->geom.compression_ratio * (trim > 0.0 ? trim : 0.0);
}

double pv_volume_cc(const EngineConfig *cfg, const CylinderConfig *cyl_cfg,
                    double local_deg) {
  return cylinder_volume_m3(local_deg, &cfg->geom, effective_cr(cfg, cyl_cfg)) *
         1.0e6;
}

static double step_deg(const EngineTrace *t, int i) {
  const double d = (double)engine_trace_at(t, i)->theta_deg -
                   (double)engine_trace_at(t, i - 1)->theta_deg;
  return fmod(d + 720.0, 720.0);
}

static int cycle_window(const EngineTrace *t, int cycles_back, int *first) {
  const int count = engine_trace_count(t);
  if (count < 2) {
    return 0;
  }

  /* skip the `cycles_back` newer cycles */
  const double skip = 720.0 * cycles_back;
  double travelled = 0.0;
  int i = count - 1;
  while (i > 0 && travelled < skip) {
    travelled += step_deg(t, i);
    i--;
  }
  if (travelled < skip) {
    return 0;
  }
  const int end = i + 1; /* one past the window's newest sample */

  travelled = 0.0;
  int n = 1;
  for (; i > 0; i--) {
    const double d = step_deg(t, i);
    if (travelled + d >= 720.0) {
      *first = end - n;
      return n;
    }
    travelled += d;
    n++;
  }
  return 0; /* ran out of trace before a whole cycle */
}

static double work_j(const PvLoop *l, int a, int b) {
  double w = 0.0;
  for (int k = a; k < b; k++) {
    const double p = 0.5 * ((double)l->p_kpa[k] + (double)l->p_kpa[k + 1]);
    const double dv = (double)l->v_cc[k + 1] - (double)l->v_cc[k];
    w += p * dv; /* kPa * cc = mJ */
  }
  return w * 1.0e-3;
}

/* First index whose angle is >= deg (l->n if none). */
static int index_at_or_after(const PvLoop *l, double deg) {
  for (int k = 0; k < l->n; k++) {
    if ((double)l->angle_deg[k] >= deg) {
      return k;
    }
  }
  return l->n;
}

static void compute_stats(const PvLoop *l, const EngineConfig *cfg,
                          const CylinderConfig *cyl_cfg, double rpm,
                          PvStats *s) {
  const double v_clear = pv_volume_cc(cfg, cyl_cfg, 0.0); /* TDC */
  const double v_bdc = pv_volume_cc(cfg, cyl_cfg, 180.0);
  s->valid = 1;
  s->v_min_cc = v_clear;
  s->v_max_cc = v_bdc;
  s->comp_ratio = v_clear > 0.0 ? v_bdc / v_clear : 0.0;
  const double disp_cc = v_bdc - v_clear;

  s->peak_kpa = 0.0;
  s->peak_angle_deg = 0.0;
  for (int k = 0; k < l->n; k++) {
    if ((double)l->p_kpa[k] > s->peak_kpa) {
      s->peak_kpa = l->p_kpa[k];
      const double a = l->angle_deg[k];
      s->peak_angle_deg = a > 360.0 ? a - 720.0 : a;
    }
  }

  const int last = l->n - 1;
  const int i180 = index_at_or_after(l, 180.0);
  const int i540 = index_at_or_after(l, 540.0);
  const double w_expansion = work_j(l, 0, i180 < l->n ? i180 : last);
  const double w_pump =
      (i180 < i540 && i540 < l->n) ? work_j(l, i180, i540) : 0.0;
  const double w_compression = i540 < l->n ? work_j(l, i540, last) : 0.0;
  const double p_close = 0.5 * ((double)l->p_kpa[last] + (double)l->p_kpa[0]);
  const double w_close =
      p_close * ((double)l->v_cc[0] - (double)l->v_cc[last]) * 1.0e-3;

  s->work_pump_j = w_pump;
  s->work_gross_j = w_expansion + w_compression + w_close;
  s->work_net_j = s->work_gross_j + s->work_pump_j;
  if (disp_cc > 0.0) {
    s->imep_net_kpa = s->work_net_j / disp_cc * 1.0e3;
    s->imep_gross_kpa = s->work_gross_j / disp_cc * 1.0e3;
    s->pmep_kpa = -s->work_pump_j / disp_cc * 1.0e3;
  } else {
    s->imep_net_kpa = s->imep_gross_kpa = s->pmep_kpa = 0.0;
  }
  s->rpm = rpm;
  s->power_kw = s->work_net_j * rpm / 120.0 * 1.0e-3;

  {
    int k = index_at_or_after(l, cfg->geom.ivc_deg);
    if (k >= l->n) {
      k = last;
    }
    s->ivc_p_kpa = l->p_kpa[k];
    s->ivc_v_cc = l->v_cc[k];
  }

  /* Least-squares slope of ln P against ln V over the compression window. */
  double lo = cfg->geom.ivc_deg + FIT_AFTER_IVC_DEG;
  if (lo < 540.0) {
    lo = 540.0;
  }
  const double hi = 720.0 - FIT_BEFORE_TDC_DEG;
  double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
  int m = 0;
  for (int k = 0; k < l->n; k++) {
    const double a = l->angle_deg[k];
    if (a < lo || a > hi || l->p_kpa[k] <= 0.0f || l->v_cc[k] <= 0.0f) {
      continue;
    }
    const double x = log((double)l->v_cc[k]);
    const double y = log((double)l->p_kpa[k]);
    sx += x;
    sy += y;
    sxx += x * x;
    sxy += x * y;
    m++;
  }
  s->compression_n = NAN;
  if (m >= FIT_MIN_POINTS) {
    const double den = m * sxx - sx * sx;
    if (fabs(den) > 1e-12) {
      s->compression_n = -(m * sxy - sx * sy) / den;
    }
  }
}

int pv_loop_build(const EngineTrace *trace, const EngineConfig *cfg,
                  const CylinderConfig *cyl_cfg, int cyl, int cycles_back,
                  PvLoop *loop, PvStats *stats) {
  loop->n = 0;
  if (stats) {
    stats->valid = 0;
  }
  if (!trace || !cfg || cyl < 0 || cyl >= ENGINE_MAX_CYLINDERS ||
      cyl >= cfg->num_cylinders || cycles_back < 0) {
    return 0;
  }

  int first = 0;
  const int n = cycle_window(trace, cycles_back, &first);
  if (n < 2) {
    return 0;
  }

  double phase[ENGINE_MAX_CYLINDERS];
  engine_cylinder_phase_offsets(cfg, phase);
  const double cr = effective_cr(cfg, cyl_cfg);

  int seam = 0;
  double prev = 0.0;
  double sum_omega = 0.0;
  for (int k = 0; k < n; k++) {
    const EngineTraceSample *s = engine_trace_at(trace, first + k);
    const double a = crank_wrap720_deg((double)s->theta_deg - phase[cyl]);
    if (k > 0 && a < prev && seam == 0) {
      seam = k;
    }
    prev = a;
    sum_omega += (double)s->omega_rad_s;
  }

  for (int k = 0; k < n; k++) {
    const EngineTraceSample *s = engine_trace_at(trace, first + (seam + k) % n);
    const double a = crank_wrap720_deg((double)s->theta_deg - phase[cyl]);
    loop->angle_deg[k] = (float)a;
    loop->v_cc[k] = (float)(cylinder_volume_m3(a, &cfg->geom, cr) * 1.0e6);
    loop->p_kpa[k] = s->cyl_pressure_kpa[cyl];
  }
  loop->n = n;

  if (stats) {
    compute_stats(loop, cfg, cyl_cfg, rad_s_to_rpm(sum_omega / n), stats);
  }
  return n;
}
