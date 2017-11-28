#include "pointcloud.h"
#include <stdlib.h>
#include <stdio.h>
#include <omp.h>

/** Crea la triangulación de la nube de puntos */
Triangle* pointcloud_triangulate(PointCloud pc, int* length, char* objfile)
{
	*length = 2*(pc.width - 1)*(pc.height - 1);

	Triangle* tris = calloc(*length, sizeof(Triangle));

	FILE* f = fopen(objfile, "w");

	for(int row = 0; row < pc.height; row++)
	{
		for(int col = 0; col < pc.width; col++)
		{
			Vector v = pc.cloud[row][col];
			fprintf(f, "v %lf %lf %lf\n", v.X, v.Y, v.Z);
		}
	}

	#pragma omp parallel for
	for(int row = 0; row < pc.height - 1; row++)
	{
		for(int col = 0; col < pc.width - 1; col++)
		{
			Vector* UL = &pc.cloud[row][col];
			Vector* UR = &pc.cloud[row][col + 1];
			Vector* DL = &pc.cloud[row + 1][col];
			Vector* DR = &pc.cloud[row + 1][col + 1];

			int16_t hUL = pc.dem[row][col];
			int16_t hUR = pc.dem[row][col + 1];
			int16_t hDL = pc.dem[row + 1][col];
			int16_t hDR = pc.dem[row + 1][col + 1];

			int iUL = 1+row*pc.width + col;
			int iUR = 1+row*pc.width + col + 1;
			int iDL = 1+(row+1)*pc.width + col;
			int iDR = 1+(row+1)*pc.width + col + 1;

			double DL_UR = vector_size_squared(vector_substracted_v(*DL, *UR));
			double UL_DR = vector_size_squared(vector_substracted_v(*UL, *DR));

			int len = 2*row*(pc.width - 1) + 2*col;

			if(DL_UR < UL_DR)
			{

				tris[len++] = (Triangle)
				{
					.p1 = DL,
					.p2 = UL,
					.p3 = UR,
					.heights = (Vector)
					{
						.X = hDL,
						.Y = hUL,
						.Z = hUR
					}
				};
				// triangle_centroid(&tris[len++]);

				tris[len] = (Triangle)
				{
					.p1 = DL,
					.p2 = UR,
					.p3 = DR,
					.heights = (Vector)
					{
						.X = hDL,
						.Y = hUR,
						.Z = hDR
					}
				};
				// triangle_centroid(&tris[len]);

				#pragma omp critical
				{
					fprintf(f, "f %d %d %d\n",iDL, iUL, iUR);
					fprintf(f, "f %d %d %d\n",iDL, iUR, iDR);
				}

			}
			else
			{
				tris[len++] = (Triangle)
				{
					.p1 = UL,
					.p2 = UR,
					.p3 = DR,
					.heights = (Vector)
					{
						.X = hUL,
						.Y = hUR,
						.Z = hDR
					}
				};
				// triangle_centroid(&tris[len++]);

				tris[len] = (Triangle)
				{
					.p1 = UL,
					.p2 = DR,
					.p3 = DL,
					.heights = (Vector)
					{
						.X = hUL,
						.Y = hDR,
						.Z = hDL
					}
				};
				// triangle_centroid(&tris[len]);
				#pragma omp critical
				{
					fprintf(f, "f %d %d %d\n",iUL, iUR, iDR);
					fprintf(f, "f %d %d %d\n",iUL, iDR, iDL);
				}
			}
		}
	}

	fclose(f);

	return tris;
}



/** Destruye la nube de puntos */
void pointcloud_destroy(PointCloud pc)
{
	for(int row = 0; row < pc.height; row++)
	{
		free(pc.cloud[row]);
	}
	free(pc.cloud);
}
