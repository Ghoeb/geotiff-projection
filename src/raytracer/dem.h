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
	/** Latitud superior */
	double start_lat;
	/** Longitud superior */
	double start_lon;
	/** Latitud por pixel */
	double lat_per_pixel;
	/** Longitud por pixel */
	double lon_per_pixel;
	/** Semieje en X */
	double a;
	/** Semieje en Y */
	double b;
	/** Semieje en Z */
	double c;
};

/** Representa un modelo de elevación digital (DEM) */
typedef struct dem DEM;

/** Obtiene el DEM a partir de un archivo */
DEM        dem_read_from_file(const char* demfile);
/** Crea un mapa cartesiano del dem, modelando la tierra como un elipsoide */
PointCloud dem_to_point_cloud(DEM dem);
/** Obtiene las coordenadas cartesianas de un punto */
Vector dem_get_point(DEM dem, double lat, double lon, double h);

Image* dem_to_img(DEM dem);

/** Libera los recursos asociados al DEM */
void       dem_destroy(DEM dem);
