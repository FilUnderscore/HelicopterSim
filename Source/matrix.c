#include "matrix.h"

#include <GLFW/glfw3.h>

#define _USE_MATH_DEFINES
#include <math.h>

void perspective_matrix(float m[16], float verticalFov, float aspectRatio, float nearZ, float farZ)
{
	float xmin, xmax, ymin, ymax;

	ymax = nearZ * tan(verticalFov * M_PI / 360.0f);
	ymin = -ymax;
	xmin = ymin * aspectRatio;
	xmax = ymax * aspectRatio;

	glFrustum(xmin, xmax, ymin, ymax, nearZ, farZ);
	
	/*
	float height = 1.0F / tanf(verticalFov * 0.5F);

	m[0] = height / aspectRatio;
	m[4] = 0.0F;
	m[8] = 0.0F;
	m[12] = 0.0F;

	m[1] = 0.0F;
	m[5] = height;
	m[9] = 0.0F;
	m[13] = 0.0F;

	m[2] = 0.0F;
	m[6] = 0.0F;
	m[10] = 0.0F; // .
	m[14] = nearZ * (1.0F - 0.0F); //

	m[3] = 0.0F;
	m[7] = 0.0F;
	m[11] = 1.0F;
	m[15] = 0.0F;

	glMultMatrixf(m);
	*/
}

void lookat_matrix(float m[16], GLfvector_t cameraPosition, GLfvector_t targetPosition, GLfvector_t upDirection)
{
	GLfvector_t forwardDirection = glfvector_normalize(create_glfvector3(targetPosition.x - cameraPosition.x, targetPosition.y - cameraPosition.y, targetPosition.z - cameraPosition.z));
	GLfvector_t rightDirection = glfvector_normalize(glfvector_cross(forwardDirection, upDirection));
	upDirection = glfvector_cross(rightDirection, forwardDirection);

	m[0] = rightDirection.x;
	m[4] = rightDirection.y;
	m[8] = rightDirection.z;
	m[12] = 0.0f;

	m[1] = upDirection.x;
	m[5] = upDirection.y;
	m[9] = upDirection.z;
	m[13] = 0.0f;

	m[2] = -forwardDirection.x;
	m[6] = -forwardDirection.y;
	m[10] = -forwardDirection.z;
	m[14] = 0.0f;

	m[3] = 0.0f;
	m[7] = 0.0f;
	m[11] = 0.0f;
	m[15] = 1.0f;
}

int invert_matrix(float m[16], float invOut[16])
{
	float inv[16], det;
	int i;

	inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
	inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
	inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
	inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
	inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
	inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
	inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
	inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	if (det == 0)
		return 0;

	det = 1.f / det;

	for (i = 0; i < 16; i++)
		invOut[i] = inv[i] * det;

	return 1;
}