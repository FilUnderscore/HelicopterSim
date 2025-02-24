#pragma once

#include "vector.h"

void perspective_matrix(float m[16], float verticalFov, float aspectRatio, float nearZ, float farZ);
void lookat_matrix(float m[16], GLfvector_t cameraPosition, GLfvector_t targetPosition, GLfvector_t upDirection);
int invert_matrix(float m[16], float invOut[16]);