#include "object.h"

object_t create_object(transform_t transform, mesh_t* mesh, void (*update)(object_t*, float))
{
	object_t object;

	object.transform = transform;
	//object.draw = draw;
	object.update = update;
	object.mesh = mesh;

	return object;
}