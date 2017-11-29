#pragma once

#include "vector.h"

struct spherical_vector
{
	double T;
	double P;
	double R;
};

typedef struct spherical_vector SVector;

/** Convierte el vector a coordenadas cartesianas */
Vector svector_to_cartesian(SVector point);

/** Convierte el vector a coordenadas esféricas */
SVector vector_to_spherical(Vector point);
