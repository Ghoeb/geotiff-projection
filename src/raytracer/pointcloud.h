#pragma once

#include "../modules/geometry.h"
#include <stdint.h>

struct pointcloud
{
	/** Nube de puntos, correspondiente a una matriz bidimensional */
	Vector** cloud;
	/** Las alturas sobre la superficie de la tierra */
	int16_t** dem;
	/** Ancho en celdas de la nube */
	int width;
	/** Alto en celdas de la nube */
	int height;
};

/** Representa una nube de puntos en el espacio tridimensional */
typedef struct pointcloud PointCloud;

/** Destruye la nube de puntos */
void pointcloud_destroy(PointCloud pc);

/** Crea la triangulación de la nube de puntos (referenciada) */
Triangle* pointcloud_triangulate(PointCloud pc, int* length, char* objfile);
