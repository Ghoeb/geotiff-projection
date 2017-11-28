#include "boundingbox.h"
#include <math.h>

/** Convierte el vector a coordenadas cartesianas */
static Vector vector_to_cartesian(Vector point)
{
	double X = point.Z * sin(point.X) * cos(point.Y);
	double Y = point.Z * sin(point.X) * sin(point.Y);
	double Z = point.Z * cos(point.X);

	return (Vector){.X = X, .Y = Y, .Z = Z};
}

/** Convierte el vector a coordenadas esféricas */
static Vector vector_to_spherical(Vector point)
{
	double radius = sqrt(vector_dot(point,point));
	double theta = acos(point.Z / radius);
	double phi = atan2(point.Y, point.X);

	return (Vector){.X = theta, .Y = phi, .Z = radius};
}

/** Indica si un punto está dentro de la caja */
static bool sbb_inside(SBB box, Vector point)
{
	Vector spherical = vector_to_spherical(point);

	if(spherical.X < box.min.X) return false;
	if(spherical.X > box.max.X) return false;
	if(spherical.Y < box.min.Y) return false;
	if(spherical.Y > box.max.Y) return false;
	if(spherical.Z < box.min.Z) return false;
	if(spherical.Z > box.max.Z) return false;

	return true;
}

/** Obtiene la normal del plano */
static Vector sbb_get_plane_normal(Vector s1, Vector s2)
{
	/* Los 3 puntos que definen el plano. El origen es obligatorio */
	Vector p0 = {.X = 0, .Y = 0, .Z = 0};
	Vector p1 = vector_normalized(vector_to_cartesian(s1));
	Vector p2 = vector_normalized(vector_to_cartesian(s2));
	/* La normal del plano */
	Vector p01 = vector_substracted_v(p1, p0);
	Vector p02 = vector_substracted_v(p2, p0);
	return vector_cross(p01, p02);
}

/** Computa los planos que envuelven a la caja */
void sbb_compute_planes(SBB* sbb)
{
	/* El vector con mínimo theta y máximo phi */
	Vector min_t_max_p = sbb -> max;
	min_t_max_p.X = sbb -> min.X;

	/* El vector con máximo theta y mínimo phi */
	Vector max_t_min_p = sbb -> min;
	max_t_min_p.X = sbb -> max.X;

	/* El plano correspondiente al mínimo theta */
	sbb -> normal_polar_min   = sbb_get_plane_normal(sbb -> min, min_t_max_p);
	/* El plano correspondiente al máximo theta */
	sbb -> normal_polar_max   = sbb_get_plane_normal(max_t_min_p, sbb -> max);
	/* El plano correspondiente al mínimo phi */
	sbb -> normal_azimuth_min = sbb_get_plane_normal(sbb -> min, max_t_min_p);
	/* El plano correspondiente al máximo phi */
	sbb -> normal_azimuth_max = sbb_get_plane_normal(min_t_max_p, sbb -> max);
}

/** Indica si el rayo choca con una de las tapas de la caja */
static bool sbb_cap_intersect(SBB box, Ray* ray, double radius, double* t_out)
{
	/* Parámetros de la ecuación cuadrática */
	double A = vector_dot(ray -> direction, ray -> direction);
	double B = 2 * vector_dot(ray -> position, ray -> direction);
	double C = vector_dot(ray -> position, ray -> position) - radius * radius;

	/* Discriminante */
	double D = (B * B) - (4 * A * C);

	/* No hay intersección */
	if(D < 0) return false;

	/* Primer caso: - */
	double t = (-B - sqrt(D))/(2*A);

	if (t > 0)
	{
		/* Calcula el punto de intersección */
		Vector point = ray -> position;
		vector_add_v(&point, vector_multiplied_f(ray -> direction, t));

		/* Si está dentro de la caja, estamos listos */
		if (sbb_inside(box, point))
		{
			*t_out = t;
			return true;
		}
	}

	/* Si existe otra solución... */
	if(D > 0)
	{
		/* Segundo caso: + */
		t = (-B + sqrt(D))/(2*A);

		/* Solo interesa lo que está adelante de */
		if(t > 0)
		{
			/* Calcula el punto de intersección */
			Vector point = ray -> position;
			vector_add_v(&point, vector_multiplied_f(ray -> direction, t));

			/* Si está dentro de la caja, estamos listos */
			if (sbb_inside(box, point))
			{
				*t_out = t;
				return true;
			}
		}
	}

	/* No quedan casos, por lo que no intersecta */
	return false;
}

/** Indica si el rayo choca con el plano que pasa por el centro */
static bool sbb_plane_intersect(SBB box, Ray* ray, Vector n, double* t_out)
{
	/* Si la dirección es perpendicular a la normal, no hay intersección */
	double denominator = vector_dot(ray -> direction, n);
	if(denominator == 0)
	{
		return false;
	}

	/* Donde intersecta el rayo al plano */
	double dividend = - vector_dot(ray -> position, n);
	double t = dividend / denominator;

	/* Solo interesa si está adelante del rayo */
	if(t < 0) return false;

	/* Calcula el punto de intersección */
	Vector point = ray -> position;
	vector_add_v(&point, vector_multiplied_f(ray -> direction, t));

	/* Si está dentro de la caja, estamos listos */
	if (sbb_inside(box, point))
	{
		*t_out = t;
		return true;
	}

	/* No está dentro de la caja */
	return false;
}

/** Indica si el rayo choca con alguna de las dos tapas de la caja */
static bool sbb_caps_intersect(SBB box, Ray* ray, double* t_out)
{
	double t_min = INFINITY;
	double t_max = INFINITY;

	bool does_min_intersect = sbb_cap_intersect(box, ray, box.min.Z, &t_min);
	bool does_max_intersect = sbb_cap_intersect(box, ray, box.max.Z, &t_max);

	*t_out = fmin(t_min, t_max);

	return does_min_intersect || does_max_intersect;
}

static bool sbb_planes_intersect(SBB box, Ray* ray, double* t_out)
{
	double t_min_polar = INFINITY;
	double t_max_polar = INFINITY;
	double t_min_azimuth = INFINITY;
	double t_max_azimuth = INFINITY;

	bool does_min_polar_intersect = sbb_plane_intersect(box, ray, box.normal_polar_min, &t_min_polar);
	bool does_max_polar_intersect = sbb_plane_intersect(box, ray, box.normal_polar_max, &t_max_polar);
	bool does_min_azimuth_intersect = sbb_plane_intersect(box, ray, box.normal_azimuth_min, &t_min_azimuth);
	bool does_max_azimuth_intersect = sbb_plane_intersect(box, ray, box.normal_azimuth_max, &t_max_azimuth);

	*t_out = fmin(t_min_polar, fmin(t_max_polar, fmin(t_min_azimuth, t_max_azimuth)));

	return does_min_polar_intersect||does_max_polar_intersect||does_min_azimuth_intersect||does_max_azimuth_intersect;
}

/** Indica si el rayo chocó con la caja */
bool sbb_intersects(SBB box, Ray* ray, double* t_out)
{
	/* Si el rayo parte dentro de la caja, entonces si o si intersecta */
	if(sbb_inside(box, ray -> position))
	{
		*t_out = 0;
		return true;
	}


	double t_caps = INFINITY;
	double t_planes = INFINITY;

	bool do_caps_intersect = sbb_caps_intersect(box, ray, &t_caps);
	bool do_planes_intersect = sbb_planes_intersect(box, ray, &t_planes);

	/* Punto donde el rayo entra a la caja */
	*t_out = fmin(t_caps, t_planes);

	return do_caps_intersect || do_planes_intersect;
}

/** Combina dos cajas */
SBB sbb_combine(SBB b1, SBB b2)
{
	SBB ret;

	ret.min.X = fmin(b1.min.X, b2.min.X);
	ret.max.X = fmax(b1.max.X, b2.max.X);
	ret.min.Y = fmin(b1.min.Y, b2.min.Y);
	ret.max.Y = fmax(b1.max.Y, b2.max.Y);
	ret.min.Z = fmin(b1.min.Z, b2.min.Z);
	ret.max.Z = fmax(b1.max.Z, b2.max.Z);

	return ret;
}
