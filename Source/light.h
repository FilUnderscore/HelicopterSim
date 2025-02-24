#pragma once

#include "vector.h"

typedef enum
{
	directional,
	spot
} lighttype_t;

typedef struct light
{
	GLfvector_t position;
	GLfvector_t color;
	GLfvector_t spotDirection;

	float intensity;
	float spotCutoffAngle;

	lighttype_t type;
	int enabled;
	int dirty;
} light_t;

void updateLightState(struct light* light, int index);
void drawLights(void);

light_t* create_light();
void init_lights();