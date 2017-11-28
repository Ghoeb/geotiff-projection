#include "dem.h"

#include <gdal.h>
#include <ogr_srs_api.h>
#include <omp.h>

#define radians(x) (((x)*M_PI)/180)

/** Obtiene el DEM a partir de un archivo */
DEM dem_read_from_file(const char* demfile)
{
	/* Registra todos los posibles drivers para leer DEMs */
  GDALAllRegister();

  /* Lee el DEM */
  GDALDatasetH hDataset = GDALOpen(demfile, GA_ReadOnly);

	if(hDataset == NULL)
  {
		abort();
  }

	/* Inicializa la estructura */
	DEM dem =
	{
		.width = GDALGetRasterXSize( hDataset ),
	  .height = GDALGetRasterYSize( hDataset )
	};

	/* Obtiene las proporciones y posiciones LAT / LON */
  /* 0 = Origen X */
  /* 1 = Tamaño de pixel en longitud (X) */
  /* 2 = ??? */
  /* 3 = Origen Y */
  /* 4 = ??? */
  /* 5 = Tamaño del pixel en latitud (Y) (ES NEGATIVO SI EL DEM ES NORTH-UP) */
	if(GDALGetGeoTransform( hDataset, dem.geoTransform) == CE_Failure)
  {
    GDALClose(hDataset);
  }


	/* Obtiene el raster */
  GDALRasterBandH hBand = GDALGetRasterBand(hDataset, 1);

	/* Obtiene la referencia espacial de la proyección */
	dem.hSpatialReference = OSRNewSpatialReference(GDALGetProjectionRef(hDataset));

  /** Matriz de alturas donde se guardará el raster, píxel por píxel */
  dem.heightmap = calloc(dem.height, sizeof(int16_t*));

  for(int row = 0; row < dem.height; row++)
  {
    dem.heightmap[row] = calloc(dem.width, sizeof(int16_t));

    /* NOTE: Mis disculpas por lo que hay a continuación */
    if (GDALRasterIO(
      hBand, /* La banda a leer (raster entero) */
      GF_Read, /* La operación que queremos hacer (lectura) */
      0, /* Coordenada X de la fila a leer */
      row, /* Coordenada Y de la fila a leer */
      dem.width, /* Ancho de la fila a leer  */
      1, /* Alto de la fila a leer */
      dem.heightmap[row], /* Buffer donde se guardará lo leído */
      dem.width, /* Ancho del buffer */
      1, /* Alto del buffer */
      GDT_Int16, /* Tipo de datos del buffer */
      0, /* Tamaño en bytes de cada pixel (usa el tipo anterior de ser 0) */
      0 /* Tamaño en bytes de una fila (lo calcula automáticamente de ser 0) */
    ) == CE_Failure)
    {
			/* Error: Liberar todo para no generar problemas */
			for(int r = 0; r <= row; r++)
			{
				free(dem.heightmap[r]);
			}
			free(dem.heightmap);
      GDALClose(hDataset);
			abort();
    }
  }

	GDALClose(hDataset);

	return dem;
}

PointCloud dem_to_point_cloud(DEM dem)
{
	double semimajor = OSRGetSemiMajor(dem.hSpatialReference, NULL);
	double semiminor = OSRGetSemiMinor(dem.hSpatialReference, NULL);

	double a = semimajor;
	double b = semimajor;
	double c = semiminor;

	/* Latitud y longitud donde parte el mapa */
	double lat_s = radians(dem.geoTransform[3]);
	double lon_s = radians(dem.geoTransform[0]);
	/* Latitud y longitud por pixel */
	double lat_per_pixel = radians(dem.geoTransform[5]);
	double lon_per_pixel = radians(dem.geoTransform[1]);

	PointCloud pc =
	{
		.height = dem.height,
		.width = dem.width
	};

	pc.cloud = calloc(pc.height, sizeof(Vector*));
	pc.dem = calloc(pc.height, sizeof(int16_t*));

	#pragma omp parallel for
	for(int row = 0; row < pc.height; row++)
	{
		double lat = lat_s + lat_per_pixel * row;

		pc.cloud[row] = calloc(pc.width, sizeof(Vector));
		pc.dem[row] = calloc(pc.height, sizeof(int16_t));

		for(int col = 0; col < pc.width; col++)
		{
			double lon = lon_s + lon_per_pixel * col;

			pc.cloud[row][col] = (Vector)
			{
				.X = a * cos(lat) * cos(lon),
				.Y = b * cos(lat) * sin(lon),
				.Z = c * sin(lat)
			};

			Vector direction = vector_normalized(pc.cloud[row][col]);

			double h = dem.heightmap[row][col];

			vector_add_v(&pc.cloud[row][col], vector_multiplied_f(direction, h));

			pc.dem[row][col] = dem.heightmap[row][col];
		}
	}

	return pc;
}

Image* dem_to_img(DEM dem)
{
	Image* img = img_png_init(dem.height, dem.width);

	double dem_max = -INFINITY;
	double dem_min = INFINITY;


	for(int row = 0; row < dem.height; row++)
	{
		for(int col = 0; col < dem.width; col++)
		{
			dem_max = fmax(dem_max, dem.heightmap[row][col]);
			dem_min = fmin(dem_min, dem.heightmap[row][col]);
		}
	}

	for(int row = 0; row < dem.height; row++)
	{
		for(int col = 0; col < dem.width; col++)
		{
			double val = dem.heightmap[row][col];
			val -= dem_min;
			val /= (dem_max - dem_min);

			img -> pixels[row][col].R = val;
			img -> pixels[row][col].G = val;
			img -> pixels[row][col].B = val;
		}
	}
	return img;
}


/** Libera los recursos asociados al DEM */
void dem_destroy(DEM dem)
{
	for(int row = 0; row < dem.height; row++)
	{
		free(dem.heightmap[row]);
	}
	free(dem.heightmap);
}
