#pragma once

#include "../modules/geometry.h"

/** Representa una caja alineada a los ejes */
struct aabb
{
	Vector min;
	Vector max;
};

/** Representa una caja alineada a los ejes */
typedef struct aabb AABB;

/** Construye una caja a partir de un arreglo de triángulos */
AABB aabb_build         (Triangle* tri, int tricount);
/** Indica si un punto esta o no dentro de una caja */
bool aabb_is_point_inside(AABB aabb, Vector p);
/** Indica si intersecta con una caja, y guarda la distancia de interseccion */
bool aabb_ray_collision (AABB aabb, Ray* ray, double* distance);
/** Indica el eje más ancho de la caja */
Axis aabb_longest_axis  (AABB aabb);

void aabb_print(AABB aabb);
