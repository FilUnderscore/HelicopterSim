#pragma once

#include "transform.h"
#include "mesh.h"

typedef struct object
{
	transform_t transform;
	mesh_t* mesh;
	void (*update)(struct object*, float);
} object_t;

object_t create_object(transform_t transform, mesh_t* mesh, void (*update)(object_t*, float));