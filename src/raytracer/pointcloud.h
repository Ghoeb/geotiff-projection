#pragma once

#include "../modules/geometry.h"
#include "../modules/sphericalvector.h"
#include <stdint.h>

struct pointcloud
{
	/** Nube de puntos, correspondiente a una matriz bidimensional */
	Vector** cloud;
	/** Nube de puntos, esta vez en coordenadas esféricas */
	SVector** spherical_cloud;
	/** Las alturas sobre la superficie de la tierra */
	int16_t** dem;
	/** Ancho en celdas de la nube */
	int width;
	/** Alto en celdas de la nube */
	int height;
};

/** Representa una nube de puntos en el espacio tridimensional */
typedef struct pointcloud PointCloud;

/** Toma solo un fragmento cuadrado de la grilla, incluyendo ambos límites */
PointCloud pc_divide(PointCloud pc, int up, int down, int left, int right);
/** Crea la triangulación de la nube de puntos (referenciada) */
Triangle* pointcloud_triangulate(PointCloud pc, int* length);
/** Destruye la nube de puntos */
void pointcloud_destroy(PointCloud pc);
