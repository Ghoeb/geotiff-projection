#pragma once

#include <stdlib.h>
#include <stdint.h>
#include "../modules/geometry.h"
#include "../imagelib/imagelib.h"
#include "pointcloud.h"
#include "aabvh.h"

struct camera
{
  /** El lugar donde estaría la imagen en el espacio : distancia focal */
  double near_clip;
  /** El ancho del sensor, en milimetros */
  double sensor_width;
  /** El alto del sensor, en milimetros */
  double sensor_height;
  /** Posición de la camara */
  Vector position;
  /** Hacia donde queda el cielo para la cámara */
  Vector up;
  /** Hacia donde mira la cámara */
  Vector target;
  /** Color de fondo de la cámara */
  Vector background_color;
  /** Altura más alta de la escena */
  double max_height;
  /** Altura más baja de la escena */
  double min_height;
};
/** Representa una cámara que capta la luz de una escena */
typedef struct camera Camera;

/** Posiciona la camara segun la nube de puntos */
Camera camera_init(PointCloud pc, double tripod, double angle);
/** Genera una imagen a partir de una escena */
// Image* camera_render(Camera camera, Triangle* tris, int tri_count);
Image* camera_render(Camera camera, AABVH* bvh, int tri_count);
