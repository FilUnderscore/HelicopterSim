#pragma once

#include "transform.h"

typedef struct object
{
	transform_t transform;
	void (*draw)(unsigned long*, struct object*);
	void (*update)(struct object*, float);
} object_t;

object_t create_object(transform_t transform, void (*draw)(unsigned long*, object_t*), void (*update)(object_t*, float));