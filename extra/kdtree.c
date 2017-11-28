#include "kdtree.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define KD_LEAF_TRIS 250000
#define KD_MAX_DEPTH 8

char axis_char(Axis ax)
{
	switch(ax)
	{
		case X: return 'X';
		case Y: return 'Y';
		case Z: return 'Z';
	}
	abort();
}

int8_t cmp_tri_plane(Triangle tri, Axis ax, double divide)
{
	double p1 = vector_get_component(*tri.p1, ax);
	double p2 = vector_get_component(*tri.p2, ax);
	double p3 = vector_get_component(*tri.p3, ax);

	double min = fmin(p1, fmin(p2,p3));
	double max = fmax(p1, fmax(p2,p3));

	if (max < divide) return -1;
	if (min > divide) return +1;
	return 0;
}

/** Construye el KDTree de los triángulos */
KDTree* kd_build(Triangle* tris, int tricount, int depth)
{
	// getchar();

	KDTree* kd = malloc(sizeof(KDTree));

	kd -> box = aabb_build(tris, tricount);

	printf("Tricount: %d\n", tricount);

	aabb_print(kd -> box);

	if(tricount < KD_LEAF_TRIS || depth > KD_MAX_DEPTH)
	{
		kd -> is_leaf = true;
		kd -> tris = tris;
		kd -> tricount = tricount;
		kd -> leftson = NULL;
		kd -> rightson = NULL;
	}
	else
	{
		kd -> is_leaf = false;
		kd -> tris = NULL;

		kd -> ax = aabb_longest_axis(kd -> box);

		printf("Axis = %c\n", axis_char(kd -> ax));

		kd -> divide = get_median(tris, tricount, kd -> ax);

		printf("Median: %lf\n", kd -> divide);

		int lesser = 0;
		int greater = 0;

		int8_t* cmp_array = calloc(tricount, sizeof(int8_t));

		for(int t = 0; t < tricount; t++)
		{
			cmp_array[t] = cmp_tri_plane(tris[t], kd -> ax, kd -> divide);
			if(cmp_array[t] <= 0)
			{
				lesser++;
			}
			if(cmp_array[t] >= 0)
			{
				greater++;
			}
		}

		printf("Lesser = %d\n", lesser);
		printf("Greater = %d\n", greater);

		Triangle* lesser_tris = calloc(lesser, sizeof(Triangle));
		Triangle* greater_tris = calloc(greater, sizeof(Triangle));

		lesser = 0;
		greater = 0;

		for(int t = 0; t < tricount; t++)
		{
			if(cmp_array[t] <= 0)
			{
				lesser_tris[lesser++] = tris[t];
			}
			if(cmp_array[t] >= 0)
			{
				greater_tris[greater++] = tris[t];
			}
		}

		free(cmp_array);

		// #pragma omp parallel
		// {
			kd -> leftson = kd_build(lesser_tris, lesser, depth + 1);
			kd -> rightson = kd_build(greater_tris, greater, depth + 1);
		// }
		free(tris);
	}
	return kd;
}



/** Recorre el arbol para intersectar el rayo con el triángulo más cercano */
bool kd_intersect_with_closest_triangle(KDTree* kd, Ray* ray)
{
	if(kd -> is_leaf)
	{
		/* Intersecta el rayo con todos los triángulos */
		for(int tri = 0; tri < kd -> tricount; tri++)
		{
			ray_intersect(ray, kd -> tris[tri]);
		}

		/* Si es que intersectó con algo */
		return ray -> did_intersect && aabb_is_point_inside(kd -> box, ray_get_intersection_point(ray));
	}
	else
	{
		double leftmin;
		bool left_in = aabb_ray_collision(kd -> leftson -> box, ray, &leftmin);

		double rightmin;
		bool right_in = aabb_ray_collision(kd -> rightson -> box, ray, &rightmin);

		if(left_in)
		{
			if(right_in)
			{
				if(leftmin < rightmin)
				{
					if(kd_intersect_with_closest_triangle(kd -> leftson, ray)) return true;
					return kd_intersect_with_closest_triangle(kd -> rightson, ray);
				}
				else
				{
					if(kd_intersect_with_closest_triangle(kd -> rightson, ray)) return true;
					return kd_intersect_with_closest_triangle(kd -> leftson, ray);
				}
			}
			return kd_intersect_with_closest_triangle(kd -> leftson, ray);
		}
		else if(right_in)
		{
			return kd_intersect_with_closest_triangle(kd -> rightson, ray);
		}
		return false;
	}
}

/** Libera los recursos asociados al árbol */
void    kd_destroy(KDTree* kd)
{
	if(kd -> is_leaf)
	{
		free(kd -> tris);
	}
	else
	{
		kd_destroy(kd -> leftson);
		kd_destroy(kd -> rightson);
	}
	free(kd);
}
