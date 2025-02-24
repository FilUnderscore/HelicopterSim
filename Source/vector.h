#pragma once

typedef struct
{
	float x, y, z, w;
} GLfvector_t;

GLfvector_t create_glfvector4(float x, float y, float z, float w);
GLfvector_t create_glfvector3(float x, float y, float z);

float glfvector_dot(GLfvector_t a, GLfvector_t b);
float glfvector_magnitude(GLfvector_t vector);

GLfvector_t glfvector_cross(GLfvector_t a, GLfvector_t b);
GLfvector_t glfvector_multiply(GLfvector_t vector, float scalar);
GLfvector_t glfvector_add(GLfvector_t a, GLfvector_t b);
GLfvector_t glfvector_normalize(GLfvector_t vector);