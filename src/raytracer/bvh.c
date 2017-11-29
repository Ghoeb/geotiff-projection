#include "bvh.h"

/** Construye el árbol y los triángulos a partir de la nube de puntos */
BVH* bvh_build(PointCloud pc)
{
	/* Cantidad de triangulos que contiene la nube de puntos */
	int length = 2*(pc.width - 1)*(pc.height - 1);
}
/** Intersecta el rayo con el triángulo más cercano dentro de la estructura */
bool bvh_intersect(BVH* bvh, Ray* ray)
{
	return false;
}
/** Libera todos los recursos asociados a la estructura */
void bvh_destroy(BVH* bvh)
{

}
