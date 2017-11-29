#pragma once

#include "../modules/boundingbox.h"
#include "pointcloud.h"

/* Representa un arbol binario de organización espacial */
struct bounding_volume_hierarchy;
/* Representa un arbol binario de organización espacial */
typedef struct bounding_volume_hierarchy BVH;

struct bounding_volume_hierarchy
{
	/** Caja que envuelve a todos los triangulos de sus hijos */
	SBB box;
	/** El hijo izquierdo de la caja */
	BVH* left_son;
	/** El hijo derecho de la caja */
	BVH* right_son;
	/** Indica si este nodo es hoja */
	bool is_leaf;
	/** Triangulos que contiene esta hoja */
	Triangle* tris;
	/** Cantidad de triángulos que contiene esta hoja */
	int tri_count;
};

/** Construye el árbol y los triángulos a partir de la nube de puntos */
BVH* bvh_build(PointCloud pc);
/** Intersecta el rayo con el triángulo más cercano dentro de la estructura */
bool bvh_intersect(BVH* bvh, Ray* ray);
/** Libera todos los recursos asociados a la estructura */
void bvh_destroy(BVH* bvh);
