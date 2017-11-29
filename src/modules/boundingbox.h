#pragma once

#include "geometry.h"
#include "sphericalvector.h"

struct sphericalboundingbox
{
	/** theta - phi - radio mínimo de la caja */
	SVector min;
	/** theta - phi - radio máximo de la caja */
	SVector max;
	/** La normal del plano azimutal que pasa por el límite mínimo de la caja */
	Vector normal_azimuth_min;
	/** La normal del plano azimutal que pasa por el límite máximo de la caja */
	Vector normal_azimuth_max;
	/** La normal del plano polar que pasa por el límite mínimo de la caja */
	Vector normal_polar_max;
	/** La normal del plano polar que pasa por el límite mínimo de la caja */
	Vector normal_polar_min;
};

typedef struct sphericalboundingbox SBB;

/** Combina dos cajas */
SBB  sbb_combine(SBB b1, SBB b2);
/** Computa los planos que envuelven a la caja */
void sbb_compute_planes(SBB* sbb);
/** Indica si el rayo chocó con la caja, y computa el punto de intersección */
bool sbb_intersects(SBB sbb, Ray* ray, double* t_closest);

void sbb_print(SBB sbb);
