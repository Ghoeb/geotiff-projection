#include "aabvh.h"

#include <math.h>
#include <stdio.h>

#define LEAF_LIMIT 100

/** Construye el árbol y los triángulos a partir de la nube de puntos */
AABVH* aabvh_build(PointCloud pc)
{
	AABVH* aabvh = malloc(sizeof(AABVH));

	/* Cantidad de triangulos que contiene la nube de puntos */
	int tri_count = 2*(pc.width - 1)*(pc.height - 1);

	/* Como máximo se permiten 100 triángulos por hoja */
	if(tri_count <= LEAF_LIMIT)
	{
		aabvh -> is_leaf = true;

		aabvh -> tris = pointcloud_triangulate(pc, &aabvh -> tri_count);

		aabvh -> box = aabb_build(aabvh -> tris, aabvh -> tri_count);

		aabvh -> left_son = NULL;
		aabvh -> right_son = NULL;
	}
	else
	{
		aabvh -> is_leaf = false;

		PointCloud leftpc;
		PointCloud rightpc;

		if(pc.width > pc.height)
		{
			leftpc = pc_divide(pc, 0, pc.height - 1, 0, pc.width/2);
			rightpc = pc_divide(pc, 0, pc.height - 1, pc.width/2, pc.width - 1);
		}
		else
		{
			leftpc = pc_divide(pc, 0, pc.height/2, 0, pc.width - 1);
			rightpc = pc_divide(pc, pc.height/2, pc.height - 1, 0, pc.width - 1);
		}

		aabvh -> left_son = aabvh_build(leftpc);
		aabvh -> right_son = aabvh_build(rightpc);

		free(leftpc.cloud);
		free(rightpc.cloud);

		free(leftpc.spherical_cloud);
		free(rightpc.spherical_cloud);

		free(leftpc.dem);
		free(rightpc.dem);

		aabvh -> box = aabb_combine(aabvh -> left_son -> box, aabvh -> right_son -> box);

		aabvh -> tri_count = aabvh -> left_son -> tri_count + aabvh -> right_son -> tri_count;
		aabvh -> tris = NULL;
	}
	return aabvh;
}
/** Intersecta el rayo con el triángulo más cercano dentro de la estructura */
bool aabvh_intersect(AABVH* aabvh, Ray* ray)
{
	if(aabvh -> is_leaf)
	{
		/* Intersecta el rayo con todos los triángulos */
		for(int tri = 0; tri < aabvh -> tri_count; tri++)
		{
			ray_intersect(ray, &aabvh -> tris[tri]);
		}

		if(ray -> closestObject)
		{
			if(aabb_is_point_inside(aabvh -> box, ray_get_intersection_point(ray)))
			{
				return true;
			}
			else
			{
				ray_reset(ray);
			}
		}
		return false;
	}
	else
	{
		double t_left = INFINITY;
		double t_right = INFINITY;

		bool does_intersect_left = aabb_ray_collision(aabvh -> left_son -> box, ray, &t_left);
		bool does_intersect_right = aabb_ray_collision(aabvh -> right_son -> box, ray, &t_right);

		/* Si intersecta el izquierdo */
		if(does_intersect_left)
		{
			/* Y además el derecho */
			if(does_intersect_right)
			{
				/* Entonces el más cercano se prueba primero */
				if(t_left < t_right)
				{
					return aabvh_intersect(aabvh -> left_son, ray) || aabvh_intersect(aabvh -> right_son, ray);
				}
				else
				{
					return aabvh_intersect(aabvh -> right_son, ray)|| aabvh_intersect(aabvh -> left_son, ray);
				}
			}
			/* Si el derecho no intersecta */
			else
			{
				return aabvh_intersect(aabvh -> left_son, ray);
			}
		}
		/* Si solo el derecho intersecta */
		else if(does_intersect_right)
		{
			return aabvh_intersect(aabvh -> right_son, ray);
		}
		return false;
	}
}
/** Libera todos los recursos asociados a la estructura */
void aabvh_destroy(AABVH* aabvh)
{
	if(aabvh -> is_leaf)
	{
		free(aabvh -> tris);
	}
	else
	{
		aabvh_destroy(aabvh -> left_son);
		aabvh_destroy(aabvh -> right_son);
	}
	free(aabvh);
}
