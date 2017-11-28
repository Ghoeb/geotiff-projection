#pragma once

#include "geometry.h"

struct sphericalboundingbox;

typedef struct sphericalboundingbox SBB;

/** Combina dos cajas */
SBB  sbb_combine(SBB b1, SBB b2);
/** Computa los planos que envuelven a la caja */
void sbb_compute_planes(SBB* sbb);
/** Indica si el rayo chocó con la caja, y computa el punto de intersección */
bool sbb_intersects(SBB sbb, Ray* ray, double* t_closest);
