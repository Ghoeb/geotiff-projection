#include "sphericalvector.h"
#include <math.h>
#include <stdlib.h>

SVector svector_init(double T, double P, double R)
{
	while(T < 0)
	{
		T += 2*M_PI;
	}
	// if(T > M_PI)
	// {
	// 	abort();
	// }
	while(P < 0)
	{
		P += 2*M_PI;
	}
	return (SVector){.T = T, .P = P, .R = R};
}

/** Convierte el vector a coordenadas cartesianas */
Vector svector_to_cartesian(SVector point)
{
	double X = point.R * sin(point.T) * cos(point.P);
	double Y = point.R * sin(point.T) * sin(point.P);
	double Z = point.R * cos(point.T);

	return (Vector){.X = X, .Y = Y, .Z = Z};
}

/** Convierte el vector a coordenadas esféricas */
SVector vector_to_spherical(Vector point)
{
	double radius = vector_size(point);
	double theta = acos(point.Z / radius);
	double phi = atan2(point.Y, point.X);

	return svector_init(theta, phi, radius);
}
