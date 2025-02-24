#pragma once

#include "object.h"

typedef struct
{
	object_t* target;
	float cameraYaw;
	float cameraPitch;
	int orbitCamera;
} camera_t;

void init_camera();
void update_camera(float dt);
void set_camera_target(object_t* target);