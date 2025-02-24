#include "light.h"

#include <GLFW/glfw3.h>

light_t lights[8];
int newLightIndex = 0;

void drawLights(void)
{
	for (int i = 0; i < (sizeof(lights) / sizeof(lights[0])); i++)
	{
		light_t* light = &lights[i];

		if (light->dirty)
		{
			updateLightState(light, i);
			light->dirty = 0;
		}

		if (!light->enabled)
		{
			continue;
		}

		float lightPosition[] = { light->position.x, light->position.y, light->position.z, (float)light->type };
		float lightDiffuseColor[] = { light->color.x * light->intensity, light->color.y * light->intensity, light->color.z * light->intensity, 1 };

		float lightAmbientColor[] = { 0, 0, 0, 1 };
		float lightSpecularColor[] = { 1, 1, 1, 1 };

		glLightfv(GL_LIGHT0 + i, GL_POSITION, lightPosition);
		glLightfv(GL_LIGHT0 + i, GL_AMBIENT, lightAmbientColor);
		glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lightDiffuseColor);
		glLightfv(GL_LIGHT0 + i, GL_SPECULAR, lightSpecularColor);

		switch (light->type)
		{
		case directional:
			break;
		case spot:
		{
			GLfloat lightSpotDirection[] = { light->spotDirection.x, light->spotDirection.y, light->spotDirection.z };
			GLfloat lightTheta = light->spotCutoffAngle;
			glLightfv(GL_LIGHT0 + i, GL_SPOT_DIRECTION, lightSpotDirection);
			glLightf(GL_LIGHT0 + i, GL_SPOT_CUTOFF, lightTheta);
			glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.01f);
			break;
		}
		}
	}
}

void updateLightState(struct light* light, int index)
{
	if (light->enabled)
	{
		glEnable(GL_LIGHT0 + index);
	}
	else
	{
		glDisable(GL_LIGHT0 + index);
	}
}

light_t* create_light()
{
	return &lights[newLightIndex++];
}

void init_lights()
{
	// Simple lighting setup
	GLfloat globalAmbient[] = { 0.2f, 0.2f, 0.2f, 1 };

	// Configure global ambient lighting.
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);

	// Enable lighting
	glEnable(GL_LIGHTING);

	// Make GL normalize the normal vectors we supply.
	glEnable(GL_NORMALIZE);

	bind_post_processing(drawLights);
}