#include "../modules/geometry.h"

typedef int (*cmp_function_t)(Triangle, Triangle);

/* http://stackoverflow.com/questions/2509679/how-to-generate-a-random-number-from-within-a-range */
unsigned int rand_interval(unsigned int min, unsigned int max)
{
    int r;
    const unsigned int range = 1 + max - min;
    const unsigned int buckets = RAND_MAX / range;
    const unsigned int limit = buckets * range;

    /* Create equal size buckets all in a row, then fire randomly towards
     * the buckets until you land in one of them. All buckets are equally
     * likely. If you land off the end of the line of buckets, try again. */
    do
    {
        r = rand();
    } while (r >= limit);

    return min + (r / buckets);
}

static void swap(Triangle* tris, int i, int j)
{
	Triangle aux = tris[i];
	tris[i] = tris[j];
	tris[j] = aux;
}

static int cmp_cmp(double cmp)
{
	if(cmp < 0) return -1;
	if(cmp > 0) return +1;
	return 0;
}

static int cmp_x_axis(Triangle t1, Triangle t2)
{
	return cmp_cmp(t1.centroid.X - t2.centroid.X);
}

static int cmp_y_axis(Triangle t1, Triangle t2)
{
	return cmp_cmp(t1.centroid.Y - t2.centroid.Y);
}

static int cmp_z_axis(Triangle t1, Triangle t2)
{
	return cmp_cmp(t1.centroid.Z - t2.centroid.Z);
}

static cmp_function_t get_cmp(Axis a)
{
	switch(a)
	{
		case X: return cmp_x_axis;
		case Y: return cmp_y_axis;
		case Z: return cmp_z_axis;
	}
	abort();
}

static int partition(Triangle* tris, int start, int end, cmp_function_t compare)
{
	if(start >= end) abort();
	int j = start-1;
	int k = end;
	Triangle v = tris[start];
	while(true)
	{
		j = j + 1;
		while(j <= end && compare(tris[j], v) < 0)
		{
			j = j + 1;
		}
		k = k-1;
		while(k >= start && compare(tris[k], v) > 0)
		{
			k = k-1;
		}
		if(j >= k) break;
		swap(tris,j,k);
	}
	swap(tris,j,end);
	return j;
}

static int random_partition(Triangle* tris, int start, int end, cmp_function_t compare)
{
	int j;
	if(end - start > 0)
	{
		j = rand_interval(start, end);
	}
	else
	{
		j = end;
	}
	if(j < start) abort();
	if(j > end) abort();
	swap(tris, j, end);
	return partition(tris, start, end, compare);
}

static void quick_select(Triangle* tris, int start, int end, int target, cmp_function_t compare)
{
	if(start < end)
	{
		int pivot = random_partition(tris, start, end, compare);

		if(pivot > target) quick_select(tris, start, pivot-1, target, compare);
		if(pivot < target) quick_select(tris, pivot + 1, end, target, compare);
	}
}

double get_median(Triangle* tris, int tricount, Axis ax)
{
  quick_select(tris, 0, tricount - 1, tricount / 2, get_cmp(ax));
  double first_median = vector_get_component(tris[tricount / 2].centroid, ax);

  if(tricount % 2 == 0)
  {
    quick_select(tris, 0, tricount - 1, tricount / 2 - 1, get_cmp(ax));
    double second_median = vector_get_component(tris[tricount / 2 - 1].centroid, ax);

    return (first_median + second_median) * 0.5;
  }

  return first_median;
}
