#include "object.h"

object_t create_object(transform_t transform, void (*draw)(unsigned long*, object_t*), void (*update)(object_t*, float))
{
	object_t object;

	object.transform = transform;
	object.draw = draw;
	object.update = update;

	return object;
}