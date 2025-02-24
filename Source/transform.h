#pragma once

#include "vector.h"
#include "quaternion.h"

typedef struct
{
	float m[16];
} transform_t;

GLfvector_t get_translation(float m[16]);
GLfvector_t get_scale(float m[16]);
GLfquaternion_t get_rotation(float m[16]);

transform_t get_transform(GLfvector_t translation, GLfvector_t rotation, GLfvector_t scale);