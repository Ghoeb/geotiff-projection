#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "pointcloud.h"
#include "../imagelib/imagelib.h"

struct dem
{
	/** Mapa de alturas */
	int16_t** heightmap;
	/** Ancho en celdas del mapa */
	int width;
	/** Alto en celdas del mapa */
	int height;
	/** Información para convertir el DEM a coordenadas de mundo */
	double geoTransform[6];
	void* hSpatialReference;
};

/** Representa un modelo de elevación digital (DEM) */
typedef struct dem DEM;

/** Obtiene el DEM a partir de un archivo */
DEM        dem_read_from_file(const char* demfile);
/** Crea un mapa cartesiano del dem, modelando la tierra como un elipsoide */
PointCloud dem_to_point_cloud(DEM dem);

Image* dem_to_img(DEM dem);

/** Libera los recursos asociados al DEM */
void       dem_destroy(DEM dem);
