#ifndef PHYSICS_COMBUSTION_H
#define PHYSICS_COMBUSTION_H

double combustion_indicated_torque_nm(double map_kpa, double omega_rad_s);

/* Indicated power implied by the current operating point (torque * omega). */
double combustion_indicated_power_w(double map_kpa, double omega_rad_s);

/* Waste heat release rate, in watts: total combustion energy release
 * implied by the indicated power at a fixed indicated thermal
 * efficiency, minus the indicated power itself. */
double combustion_waste_heat_w(double map_kpa, double omega_rad_s);

/* Engine load as a fraction of rated indicated power, clamped to [0, 1].
 * 0 when the crank is stopped. Drives the (saturating) CHT/EGT targets so
 * gas/metal temperatures don't scale without bound with absolute power. */
double combustion_load_fraction(double map_kpa, double omega_rad_s);

#endif /* PHYSICS_COMBUSTION_H */
