#pragma once

#include "../modules/geometry.h"
#include "aabb.h"

struct kdtree;

typedef struct kdtree KDTree;

struct kdtree
{
	/** Elementos menores al eje */
	KDTree* leftson;
	/** Elementos mayores al eje */
	KDTree* rightson;
	/** Caja que envuelve a todos los elementos de este arbol */
	AABB box;
	/** Eje según el cual este arbol divide */
	Axis ax;
	/** Plano segun el cual se divide */
	double divide;
	/** Indica que este arbol es una hoja */
	bool is_leaf;
	/** Arreglo de triangulos que contiene este nodo */
	Triangle* tris;
	/** Cantidad de triángulos que tiene este nodo */
	int tricount;
};

/** Obtiene la mediana de un arreglo de triángulos */
double  get_median(Triangle* tris, int tricount, Axis ax);
/** Construye el KDTree de los triángulos */
KDTree* kd_build(Triangle* tris, int tricount, int depth);
/** Recorre el arbol para intersectar el rayo con el triángulo más cercano */
bool    kd_intersect_with_closest_triangle(KDTree* kd, Ray* ray);
/** Libera los recursos asociados al árbol */
void    kd_destroy(KDTree* kd);
