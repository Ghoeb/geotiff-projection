#include "bvh.h"
#include <math.h>

#define LEAF_LIMIT 100

/* Intercambia dos valores de tipo double */
void double_swap(double* x, double* y)
{
	double aux = *x;
	*x = *y;
	*y = aux;
}

SBB sbb_build(PointCloud pc)
{
	SBB box =
	{
		.min.T = pc.spherical_cloud[0][0].T,
		.max.T = pc.spherical_cloud[pc.height - 1][0].T,
		.min.P = pc.spherical_cloud[0][0].P,
		.max.P = pc.spherical_cloud[0][pc.width - 1].P,
		.min.R = pc.spherical_cloud[0][0].R,
		.max.R = pc.spherical_cloud[0][0].R
	};

	if(box.min.T > box.max.T)	double_swap(&box.min.T, &box.max.T);
	if(box.min.P > box.max.P)	double_swap(&box.min.P, &box.max.P);

	for(int row = 0; row < pc.height; row++)
	{
		for(int col = 0; col < pc.width; col++)
		{
			box.min.R = fmin(pc.spherical_cloud[row][col].R, box.min.R);
			box.max.R = fmax(pc.spherical_cloud[row][col].R, box.max.R);
		}
	}
	return box;
}

/** Construye el árbol y los triángulos a partir de la nube de puntos */
BVH* bvh_build(PointCloud pc)
{
	BVH* bvh = malloc(sizeof(BVH));

	/* Cantidad de triangulos que contiene la nube de puntos */
	int tri_count = 2*(pc.width - 1)*(pc.height - 1);

	/* Como máximo se permiten 100 triángulos por hoja */
	if(tri_count <= LEAF_LIMIT)
	{
		bvh -> is_leaf = true;

		bvh -> box = sbb_build(pc);
		sbb_compute_planes(&bvh -> box);

		bvh -> tris = pointcloud_triangulate(pc, &bvh -> tri_count);

		bvh -> left_son = NULL;
		bvh -> right_son = NULL;
	}
	/* Hay que dividir */
	else
	{
		bvh -> is_leaf = false;

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

		bvh -> left_son = bvh_build(leftpc);
		bvh -> right_son = bvh_build(rightpc);

		free(leftpc.cloud);
		free(rightpc.cloud);

		free(leftpc.spherical_cloud);
		free(rightpc.spherical_cloud);

		free(leftpc.dem);
		free(rightpc.dem);

		bvh -> box = sbb_combine(bvh -> left_son -> box, bvh -> right_son -> box);
	}
	return bvh;
}

/** Intersecta el rayo con el triángulo más cercano dentro de la estructura */
bool bvh_intersect(BVH* bvh, Ray* ray)
{
	if(bvh -> is_leaf)
	{
		/* Intersecta el rayo con todos los triángulos */
		for(int tri = 0; tri < bvh -> tri_count; tri++)
		{
			ray_intersect(ray, &bvh -> tris[tri]);
		}

		return ray -> closestObject;
	}
	else
	{
		double t_left = INFINITY;
		double t_right = INFINITY;

		bool does_intersect_left = sbb_intersects(bvh -> left_son -> box, ray, &t_left);
		bool does_intersect_right = sbb_intersects(bvh -> right_son -> box, ray, &t_right);

		/* Si intersecta el izquierdo */
		if(does_intersect_left)
		{
			/* Y además el derecho */
			if(does_intersect_right)
			{
				/* Entonces el más cercano se prueba primero */
				if(t_left < t_right)
				{
					return bvh_intersect(bvh -> left_son, ray) || bvh_intersect(bvh -> right_son, ray);
				}
				else
				{
					return bvh_intersect(bvh -> right_son, ray)|| bvh_intersect(bvh -> left_son, ray);
				}
			}
			/* Si el derecho no intersecta */
			else
			{
				return bvh_intersect(bvh -> left_son, ray);
			}
		}
		/* Si solo el derecho intersecta */
		else if(does_intersect_right)
		{
			return bvh_intersect(bvh -> right_son, ray);
		}
		return false;
	}
}

/** Libera todos los recursos asociados a la estructura */
void bvh_destroy(BVH* bvh)
{
	if(bvh -> is_leaf)
	{
		free(bvh -> tris);
	}
	else
	{
		bvh_destroy(bvh -> left_son);
		bvh_destroy(bvh -> right_son);
	}
	free(bvh);
}
