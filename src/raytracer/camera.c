#include "camera.h"
#include <stdio.h>
#include <math.h>
#include <omp.h>
// #include "kdtree.h"

#define radians(x) (((x)*M_PI)/180)
#define degrees(x) (((x)*180)/M_PI)

static Color vector_to_color(Vector v)
{
	return (Color){v.X, v.Y, v.Z};
}

// static Vector ray_trace(Ray* ray, Triangle* tris, int tri_count, Camera cam)
// {
// 	/* Intersecta el rayo con todos los triángulos */
// 	for(int tri = 0; tri < tri_count; tri++)
// 	{
// 		ray_intersect(ray, &tris[tri]);
// 	}
//
// 	// if(aabb_ray_collision(kd -> box, ray, NULL) && kd_intersect_with_closest_triangle(kd, ray))
// 	// {
//
// 	/* Si es que intersectó con algo */
// 	if(ray -> closestObject)
// 	{
// 		Vector bary = ray_get_barycentric(ray);
//
// 		Triangle* tri = ray -> closestObject;
//
// 		/* Pinta segun la altura del punto intersectado */
// 		double height = vector_dot(bary, tri -> heights);
//
// 		double value = height/cam.max_height;
//
// 		return vector_init(value, value, value);
// 	}
//
// 	return cam.background_color;
// }

static Vector ray_trace(Ray* ray, AABVH* aabvh, int tri_count, Camera cam)
{
	double t = -1;

	/* Intersecta el rayo con todos los triángulos */
	// for(int tri = 0; tri < tri_count; tri++)
	// {
	// 	ray_intersect(ray, &tris[tri]);
	// }

	// if(aabb_ray_collision(kd -> box, ray, NULL) && kd_intersect_with_closest_triangle(kd, ray))
	// {

	/* Si es que intersectó con algo */
	if(aabb_ray_collision(aabvh -> box, ray, &t) && aabvh_intersect(aabvh, ray))
	{
		Vector bary = ray_get_barycentric(ray);

		Triangle* tri = ray -> closestObject;

		/* Pinta segun la altura del punto intersectado */
		double height = vector_dot(bary, tri -> heights);

		double value = height/cam.max_height;

		return vector_init(value, value, value);
	}

	return cam.background_color;
}

static Color get_pixel_color(size_t img_x, size_t img_y, Camera camera, int height, int width, AABVH* aabvh, int tri_count)
{
  /* Convertimos las coordenadas de la camara a coordenadas reales */
	double top = camera.sensor_height / 2;
	double right = camera.sensor_width / 2;

  float unit = 0.5;

  /* Seteamos las coordenadas de la camara (u,v,w) */
  Vector W = vector_substracted_v(camera.target, camera.position);
  vector_normalize(&W);

  Vector U = vector_cross(W, camera.up);
  vector_normalize(&U);

  Vector V = vector_cross(W, U);
  vector_normalize(&V);

  float dx = unit;
  float dy = unit;

  float i = (2 * right * (img_x + dx)) / width - right;
  float j = (2 * top * (img_y + dy)) / height - top;

  /* Obtenemos las coordenadas reales */
  Vector dir_w = vector_multiplied_f(W, camera.near_clip);
  Vector dir_u = vector_multiplied_f(U, i);
  Vector dir_v = vector_multiplied_f(V, j);

  Vector Sij = {.X = 0, .Y = 0, .Z = 0};
  vector_add_v(&Sij, camera.position);
  vector_add_v(&Sij, dir_u);
  vector_add_v(&Sij, dir_v);
  vector_add_v(&Sij, dir_w);

  /* Obtenemos la direccion de la camara al punto */
  Vector Dij = vector_substracted_v(Sij, camera.position);
  vector_normalize(&Dij);

  /* Creamos el rayo de la camara al pixel */
  Ray ray = ray_create(camera.position, Dij);

	Vector pix_color = ray_trace(&ray, aabvh, tri_count, camera);

  vector_clamp(&pix_color, 0, 1);

  return vector_to_color(pix_color);
}

Image* camera_render(Camera camera, AABVH* aabvh, int tri_count)
{
	size_t h = camera.sensor_height * 10000;
	size_t w = camera.sensor_width * 10000;

	Image* img = img_png_init(h, w);

	printf("Generating %zux%zu image\n",h,w);

	// KDTree* kd = kd_build(tris, tri_count, 0);

	// printf("Built KD-Tree\n");

	double progress = 0;

	#pragma omp parallel for
	for(int y = 0; y < h; y++)
	{
		for(int x = 0; x < w; x++)
		{
			img -> pixels[y][x] = get_pixel_color(x, y, camera, h, w, aabvh, tri_count);
		}

		#pragma omp critical
		{
			progress += 100/(double)h;
			printf("%lf%%\n", progress);
		}
	}

	// kd_destroy(kd);

	return img;
}

/** Posiciona la cámara segun la nube de puntos */
Camera camera_init(PointCloud pc, double tripod, double angle)
{
	int real_x = pc.width - 1;
	int real_y = pc.height/2;

	// while(pc.dem[real_y][real_x - 1] > pc.dem[real_y][real_x])
	// {
	// 	real_x--;
	// }

	Vector p = pc.cloud[real_y][real_x];
	Vector pn = vector_normalized(p);

	double max_height = -INFINITY;

	for(int row = 0; row < pc.height; row++)
	{
		for(int col = 0; col < pc.width; col++)
		{
			max_height = fmax(max_height, pc.dem[row][col]);
		}
	}

	// Todo lo siguiente es para poder hacer que la camara apunte 10° hacia "abajo"

	Vector t = vector_normalized(pc.cloud[pc.height/2][pc.width/2]);

	Vector south = vector_substracted_v(pc.cloud[pc.height - 1][real_x], p);
	Vector west = vector_cross(south, pn);

	double angle_p_t = vector_angle(p, t);

	Vector q = vector_substracted_v(p, t);

	double angle_q_p = vector_angle(p, q);

	double angle_q_t = M_PI - angle_p_t - angle_q_p;

	double dq = vector_size(q);
	double dt = vector_size(t);

	double angle_q_rt = M_PI/2 - angle_q_p - radians(angle);

	double angle_q_x = M_PI - angle_q_t;

	double angle_x_rt = M_PI - angle_q_x - angle_q_rt;

	double dx = (sin(angle_q_rt) * dq) / sin(angle_x_rt);
	double dxt = dt + dx;

	Vector position = vector_added_v(p, vector_multiplied_f(pn, tripod));

	// Vector target = vector_added_v(vector_substracted_v(p, pn), vector_multiplied_f(prototarget, x));

	Vector target = vector_multiplied_f(vector_normalized(t), dxt);

	Vector ct = vector_substracted_v(target, p);

	printf("Angle is %lf°\n", degrees(vector_angle(ct, west)));

	Camera cam =
	{
		.sensor_width = 0.036,
		.sensor_height = 0.024,
		.near_clip = 0.050,
		.position = position,
		.up = pn,
		.target = target,
		.background_color = vector_init(0.6784,0.847,0.902),
		.max_height = max_height,
		.min_height = 0
	};

	return cam;
}
