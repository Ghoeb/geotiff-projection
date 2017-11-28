#pragma once

#include "vector.h"
#include <stdbool.h>
#include <stdlib.h>

struct triangle
{
  /** Primer vértice */
  Vector const* p1;
  /** Segundo vértice */
  Vector const* p2;
  /** Tercer vértice */
  Vector const* p3;
  /** Centro del triángulo */
  // Vector centroid;
  /** Alturas de los puntos en el espacio */
  Vector heights;
};

/** Representa un triángulo. Sus vértices son inmutables */
typedef struct triangle Triangle;

struct ray
{
  /** Origen del rayo */
  Vector position;
  /** Dirección del rayo. Debe de ser unitario. */
  Vector direction;
  /** Distancia de intersección al closestObject */
  double closestDistance;
  /** Indica que el rayo efectivamente intersectó */
  bool did_intersect;

  Triangle* closestObject;
};

/** Representa un rayo. Almacena información geométrica del rayo,
    y de intersección con objetos de la escena. */
typedef struct ray Ray;


void triangle_centroid(Triangle* tri);

/** Crea un rayo nuevo que parte desde position y va hacia direction */
Ray ray_create(Vector position, Vector direction);

/** Resetea el rayo en caso de querer descartar las colisiones guardadas */
void ray_reset(Ray* ray);

/** Intenta intersectar un rayo con un triángulo
    Si es exitoso, guarda información del éxito. Se retorna TRUE.
    Si no es exitoso, no se almacena nada y retorna FALSE. */
void ray_intersect(Ray *ray, Triangle* tri);
/** Calcula el punto de intersección en función de la distancia a la que intersectó */
Vector ray_get_intersection_point(Ray* ray);

Vector ray_get_barycentric(Ray* ray);
