#include "vector.h"

#include <math.h>

GLfvector_t create_glfvector4(float x, float y, float z, float w)
{
	GLfvector_t glfvector = { 0 };

	glfvector.x = x;
	glfvector.y = y;
	glfvector.z = z;
	glfvector.w = w;

	return glfvector;
}

GLfvector_t create_glfvector3(float x, float y, float z)
{
	return create_glfvector4(x, y, z, 0);
}

float glfvector_dot(GLfvector_t a, GLfvector_t b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float glfvector_magnitude(GLfvector_t vector)
{
	return sqrtf(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

GLfvector_t glfvector_cross(GLfvector_t a, GLfvector_t b)
{
	return create_glfvector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

GLfvector_t glfvector_multiply(GLfvector_t vector, float scalar)
{
	return create_glfvector3(vector.x * scalar, vector.y * scalar, vector.z * scalar);
}

GLfvector_t glfvector_add(GLfvector_t a, GLfvector_t b)
{
	return create_glfvector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

GLfvector_t glfvector_normalize(GLfvector_t vector)
{
	float magnitude = glfvector_magnitude(vector);

	return create_glfvector3(vector.x / magnitude, vector.y / magnitude, vector.z / magnitude);
}