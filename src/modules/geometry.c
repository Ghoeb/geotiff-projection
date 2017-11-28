#include "geometry.h"
#include <stdio.h>
#include <math.h>

// Para cortar por lo sano
#define RAY_EPSILON 0.01
// #define DISTANCE_EPSILON 0.1
#define PLANE_EPSILON 0.000001
#define DISTANCE_EPSILON PLANE_EPSILON

// void triangle_centroid(Triangle* tri)
// {
//   tri -> centroid.X = (tri -> p1 -> X + tri -> p2 -> X + tri -> p3 -> X)/3;
//   tri -> centroid.Y = (tri -> p1 -> Y + tri -> p2 -> Y + tri -> p3 -> Y)/3;
//   tri -> centroid.Z = (tri -> p1 -> Z + tri -> p2 -> Z + tri -> p3 -> Z)/3;
// }

/** Crea un rayo nuevo que parte desde position y va hacia direction */
Ray ray_create(Vector position, Vector direction)
{
  Ray ray;
  ray.position = position;
  // vector_add_v(&ray.position, vector_multiplied_f(direction, RAY_EPSILON));
  ray.direction = direction;
  ray.closestDistance = INFINITY;
  ray.did_intersect = false;
  return ray;
}

/** Resetea el rayo en caso de querer descartar las colisiones guardadas */
void ray_reset(Ray* ray)
{
  ray -> closestDistance = INFINITY;
}

void ray_intersect(Ray *ray, Triangle* tri)
{
  Vector e12 = vector_p_substracted_v(tri -> p2, tri -> p1);
  Vector e13 = vector_p_substracted_v(tri -> p3, tri -> p1);

  Vector p = vector_cross(ray -> direction, e13);

  // If the determinant is close to 0,
  // the ray lies in the plane of the triangle.
  float determinant = vector_dot(e12, p);

  // In such a case, we end the process right away
  // if(determinant > -PLANE_EPSILON && determinant < PLANE_EPSILON)
  if(determinant == 0)
    return; //false

  float invDet = 1.f / determinant;

  Vector t = vector_p_substracted_v(&ray -> position, tri -> p1);

  float u = vector_dot(t, p) * invDet;

  // If the intersection is outside of the triangle,
  // then u does not belong to [0, 1]
  if(u < 0.f || u > 1.f)
    return;//false

  Vector q = vector_cross(t, e12);

  float v = vector_dot(ray -> direction, q) * invDet;

  // Same for v. v has to be between 0 and 1
  // But also u + v has to be at most 1
  if(v < 0.f || u + v > 1.f)
    return; //false

  float distance = vector_dot(e13, q) * invDet;

  // if(distance > DISTANCE_EPSILON && distance < ray -> closestDistance)
  if(distance > 0 && distance < ray -> closestDistance)
  {
    ray -> closestDistance = distance;
    ray -> closestObject = tri;
    return; //true
  }

  return; //false
}

/** Calcula el punto de intersección en función de la distancia a la que intersectó */
Vector ray_get_intersection_point(Ray* ray)
{
  Vector point = ray -> position;
  vector_add_v(&point, vector_multiplied_f(ray -> direction, ray -> closestDistance));
  return point;
}

Vector ray_get_barycentric(Ray* ray)
{
  Triangle* tri = ray -> closestObject;

  Vector pointInTri = vector_multiplied_f(ray -> direction, ray -> closestDistance);

  pointInTri = vector_added_v(pointInTri, ray -> position);

  Vector ab = vector_p_substracted_v(tri -> p2, tri -> p1);
  Vector ac = vector_p_substracted_v(tri -> p3, tri -> p1);

  float inverseAreaTri = 1.f / vector_size(vector_cross(ab, ac));

  Vector pa = vector_p_substracted_v(tri -> p1, &pointInTri);
  Vector pb = vector_p_substracted_v(tri -> p2, &pointInTri);
  Vector pc = vector_p_substracted_v(tri -> p3, &pointInTri);

  float alpha = vector_size(vector_cross(pb, pc)) * inverseAreaTri;
  float beta = vector_size(vector_cross(pc, pa)) * inverseAreaTri;

  return (Vector){.X = alpha, .Y = beta, .Z = 1 - alpha - beta};
}
