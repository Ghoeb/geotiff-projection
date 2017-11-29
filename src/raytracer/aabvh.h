#pragma once

#include "../modules/aabb.h"
#include "pointcloud.h"

/* Representa un arbol binario de organización espacial */
struct axis_aligned_bounding_volume_hierarchy;
/* Representa un arbol binario de organización espacial */
typedef struct axis_aligned_bounding_volume_hierarchy AABVH;

struct axis_aligned_bounding_volume_hierarchy
{
	/** Caja que envuelve a todos los triangulos de sus hijos */
	AABB box;
	/** El hijo izquierdo de la caja */
	AABVH* left_son;
	/** El hijo derecho de la caja */
	AABVH* right_son;
	/** Indica si este nodo es hoja */
	bool is_leaf;
	/** Triangulos que contiene esta hoja */
	Triangle* tris;
	/** Cantidad de triángulos que contiene esta hoja */
	int tri_count;
};

/** Construye el árbol y los triángulos a partir de la nube de puntos */
AABVH* aabvh_build(PointCloud pc);
/** Intersecta el rayo con el triángulo más cercano dentro de la estructura */
bool aabvh_intersect(AABVH* aabvh, Ray* ray);
/** Libera todos los recursos asociados a la estructura */
void aabvh_destroy(AABVH* aabvh);
