#include "quaternion.h"

#include <math.h>

GLfquaternion_t create_quaternion_axis_angle(float x, float y, float z, float angle)
{
	GLfquaternion_t quaternion;
	quaternion.v = create_glfvector4(x * sinf(angle / 2), y * sinf(angle / 2), z * sinf(angle / 2), cosf(angle / 2));

	return quaternion;
}

GLfvector_t multiply_vector_by_quaternion(GLfvector_t vector, GLfquaternion_t quaternion)
{
	GLfvector_t u = create_glfvector3(quaternion.v.x, quaternion.v.y, quaternion.v.z);
	float s = quaternion.v.w;

	float a = 2.0f * glfvector_dot(u, vector);
	GLfvector_t b = glfvector_multiply(u, a);

	float c = s * s - glfvector_dot(u, u);
	GLfvector_t d = glfvector_multiply(vector, c);

	float e = 2.0f * s;
	GLfvector_t f = glfvector_cross(u, vector);
	GLfvector_t g = glfvector_multiply(f, e);

	GLfvector_t h = glfvector_add(b, d);
	GLfvector_t i = glfvector_add(h, g);

	return i;
}