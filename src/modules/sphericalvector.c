#include "sphericalvector.h"
#include <math.h>
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
	double radius = sqrt(vector_dot(point,point));
	double theta = acos(point.Z / radius);
	double phi = atan2(point.Y, point.X);

	return (SVector){.T = theta, .P = phi, .R = radius};
}
