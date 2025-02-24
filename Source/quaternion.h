#pragma once

#include "vector.h"

typedef struct
{
	GLfvector_t v;
} GLfquaternion_t;

GLfquaternion_t create_quaternion_axis_angle(float x, float y, float z, float angle);
GLfvector_t multiply_vector_by_quaternion(GLfvector_t vector, GLfquaternion_t quaternion);