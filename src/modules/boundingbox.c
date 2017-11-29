#include "boundingbox.h"
#include <math.h>



/** Indica si un punto está dentro de la caja */
static bool sbb_inside(SBB box, Vector point)
{
	SVector spherical = vector_to_spherical(point);

	if(spherical.T < box.min.T) return false;
	if(spherical.T > box.max.T) return false;
	if(spherical.P < box.min.P) return false;
	if(spherical.P > box.max.P) return false;
	if(spherical.R < box.min.R) return false;
	if(spherical.R > box.max.R) return false;

	return true;
}

/** Obtiene la normal del plano */
static Vector sbb_get_plane_normal(SVector s1, SVector s2)
{
	/* Los 3 puntos que definen el plano. El origen es obligatorio */
	Vector p0 = {.X = 0, .Y = 0, .Z = 0};
	Vector p1 = vector_normalized(svector_to_cartesian(s1));
	Vector p2 = vector_normalized(svector_to_cartesian(s2));
	/* La normal del plano */
	Vector p01 = vector_substracted_v(p1, p0);
	Vector p02 = vector_substracted_v(p2, p0);
	return vector_cross(p01, p02);
}

/** Computa los planos que envuelven a la caja */
void sbb_compute_planes(SBB* sbb)
{
	/* El vector con mínimo theta y máximo phi */
	SVector min_t_max_p = sbb -> max;
	min_t_max_p.T = sbb -> min.T;

	/* El vector con máximo theta y mínimo phi */
	SVector max_t_min_p = sbb -> min;
	max_t_min_p.T = sbb -> max.T;

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

	bool does_min_intersect = sbb_cap_intersect(box, ray, box.min.R, &t_min);
	bool does_max_intersect = sbb_cap_intersect(box, ray, box.max.R, &t_max);

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

	ret.min.T = fmin(b1.min.T, b2.min.T);
	ret.max.T = fmax(b1.max.T, b2.max.T);
	ret.min.P = fmin(b1.min.P, b2.min.P);
	ret.max.P = fmax(b1.max.P, b2.max.P);
	ret.min.R = fmin(b1.min.R, b2.min.R);
	ret.max.R = fmax(b1.max.R, b2.max.R);

	return ret;
}
