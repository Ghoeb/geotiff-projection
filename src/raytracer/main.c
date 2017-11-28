#include <gdal.h>
#include "../imagelib/imagelib.h"
#include "dem.h"
#include "camera.h"
#include <omp.h>

int main(int argc, char *argv[])
{
  if(argc != 6)
  {
    printf("Uso: %s <DEM.tif> <tri.obj> <img.png> <tripod> <angle>\ndonde\n", argv[0]);
    printf("\tDEM es el archivo en del DEM (un GeoTIFF)\n");
    printf("\ttri es el archivo donde se guardará la triangulación del DEM\n");
    printf("\timg es la imagen generada\n");
    printf("\ttripod es la altura del trípode\n");
    printf("\tangle es el angulo de la cámara hacia abajo\n");
    return 1;
  }

  omp_set_num_threads(omp_get_max_threads());

  DEM dem = dem_read_from_file(argv[1]);

  printf("Read the DEM\n");

  // printf("Image size is %d x %d\n", dem.height, dem.width);

  PointCloud pc = dem_to_point_cloud(dem);


  printf("Converted DEM to point cloud\n");

  int length;

  Triangle* tris = pointcloud_triangulate(pc, &length, argv[2]);

  printf("Triangulated point cloud\n");

  // printf("There are %d triangles\n", length);

  Camera cam = camera_init(pc, atoi(argv[4]), atof(argv[5]));

  Image* img = camera_render(cam, tris, length);

  img_png_write_to_file(img, argv[3]);

  img_png_destroy(img);

  free(tris);

  pointcloud_destroy(pc);

  dem_destroy(dem);

	return 0;
}
