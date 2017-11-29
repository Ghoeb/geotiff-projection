#include "aabb.h"
#include <math.h>
#include <stdio.h>

/** Entrega una caja vacía */
static AABB aabb_empty()
{
	return (AABB)
	{
		.min =
		{
			.X = INFINITY,
			.Y = INFINITY,
			.Z = INFINITY
		},
		.max =
		{
			.X = -INFINITY,
			.Y = -INFINITY,
			.Z = -INFINITY
		}
	};
}

/** Construye una caja a partir de un arreglo de triángulos */
AABB aabb_build (Triangle* tris, int tricount)
{
	AABB aabb = aabb_empty();

	for(int t = 0; t < tricount; t++)
	{
		Vector p1 = *tris[t].p1;
		Vector p2 = *tris[t].p2;
		Vector p3 = *tris[t].p3;

		aabb.min.X = fmin(aabb.min.X, fmin(p1.X, fmin(p2.X, p3.X)));
		aabb.min.Y = fmin(aabb.min.Y, fmin(p1.Y, fmin(p2.Y, p3.Y)));
		aabb.min.Z = fmin(aabb.min.Z, fmin(p1.Z, fmin(p2.Z, p3.Z)));

		aabb.max.X = fmax(aabb.max.X, fmax(p1.X, fmax(p2.X, p3.X)));
		aabb.max.Y = fmax(aabb.max.Y, fmax(p1.Y, fmax(p2.Y, p3.Y)));
		aabb.max.Z = fmax(aabb.max.Z, fmax(p1.Z, fmax(p2.Z, p3.Z)));
	}

	return aabb;
}

/** Indica si un punto esta o no dentro de una caja */
bool aabb_is_point_inside(AABB aabb, Vector p)
{
	if(p.X < aabb.min.X || p.X > aabb.max.X) return false;
	if(p.Y < aabb.min.Y || p.Y > aabb.max.Y) return false;
	if(p.Z < aabb.min.Z || p.Z > aabb.max.Z) return false;
	return true;
}

/** Indica si intersecta con una caja, y guarda la distancia de interseccion */
bool aabb_ray_collision (AABB aabb, Ray* ray, double* distance)
{
	double tx1 = (aabb.min.X - ray -> position.X) / ray -> direction.X;
	double tx2 = (aabb.max.X - ray -> position.X) / ray -> direction.X;

	double tmin = fmin(tx1, tx2);
	double tmax = fmax(tx1, tx2);

	double ty1 = (aabb.min.Y - ray -> position.Y) / ray -> direction.Y;
	double ty2 = (aabb.max.Y - ray -> position.Y) / ray -> direction.Y;

	tmin = fmax(tmin, fmin(ty1, ty2));
	tmax = fmin(tmax, fmax(ty1, ty2));

	double tz1 = (aabb.min.Z - ray -> position.Z) / ray -> direction.Z;
	double tz2 = (aabb.max.Z - ray -> position.Z) / ray -> direction.Z;

	tmin = fmax(tmin, fmin(tz1, tz2));
	tmax = fmin(tmax, fmax(tz1, tz2));
	if (tmax >= tmin)
	{
		if(distance)
			*distance = tmin;
	  return true;
	}
	return false;
}

void aabb_print(AABB aabb)
{
	printf("[\n\t");
	vector_print(aabb.min);
	printf("\t");
	vector_print(aabb.max);
	printf("]\n");
}

/** Combina dos cajas, creando una nueva caja que contiene ambas */
AABB aabb_combine(AABB aabba, AABB aabbb)
{
	AABB ret;
	ret.min.X = fmin(aabba.min.X, aabbb.min.X);
	ret.max.X = fmax(aabba.max.X, aabbb.max.X);
	ret.min.Y = fmin(aabba.min.Y, aabbb.min.Y);
	ret.max.Y = fmax(aabba.max.Y, aabbb.max.Y);
	ret.min.Z = fmin(aabba.min.Z, aabbb.min.Z);
	ret.max.Z = fmax(aabba.max.Z, aabbb.max.Z);
	return ret;
}

/** Indica el eje más ancho de la caja */
Axis aabb_longest_axis  (AABB aabb)
{
	Vector diff = vector_substracted_v(aabb.min, aabb.max);

	diff.X = fabs(diff.X);
	diff.Y = fabs(diff.Y);
	diff.Z = fabs(diff.Z);

	if(diff.X > diff.Y && diff.X > diff.Z) return X;
	else if(diff.Y > diff.Z) return Y;
	return Z;
}
