#pragma once

#include "scenegraph.h"
#include "texture.h"
#include "vector.h"
#include "mesh.h"

typedef struct terrain
{
	object_t object;
	texture_t* texture;
	int width, height;
	int usePerlin;
	mesh_t* mesh;
} terrain_t;

terrain_t* create_terrain(int width, int height, texture_t* texture);
void draw_terrain(unsigned long* displayList, object_t* object);

float get_terrain_height(terrain_t* terrain, float x, float z);
GLfvector_t get_random_point_on_terrain(terrain_t* terrain);