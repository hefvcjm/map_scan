#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "amcl/map/map.h"


// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}


// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}


// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)
{
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);
  
  if (!MAP_VALID(map, i, j))
    return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

