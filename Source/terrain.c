#include "terrain.h"

#include <GLFW/glfw3.h>
#include <stdlib.h>

#include "perlin.h"

terrain_t* create_terrain(int width, int height, texture_t* texture)
{
	terrain_t* terrain = (terrain_t*)calloc(1, sizeof(terrain_t));
	terrain->object.transform = get_transform(create_glfvector3(0, 0, 0), create_glfvector3(0, 0, 0), create_glfvector3(1, 1, 1));
	terrain->texture = texture;
	terrain->width = width;
	terrain->height = height;
	terrain->usePerlin = 1;

	float vertices[] = {
		-0.5f, -0.5f, 0.0f,
		0.5f, -0.5f, 0.0f,
		0.0f,  0.5f, 0.0f
	};

	unsigned int indices[] = {
		0, 1, 2
	};

	terrain->mesh = create_mesh(vertices, 9, indices, 3);

	return terrain;
}

void draw_terrain(unsigned long* displayList, object_t* object)
{
	terrain_t* terrain = (terrain_t*)object;

	//glEnable(GL_TEXTURE_2D);

	//draw_texture(terrain->texture);

	if (*displayList == -1)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		int width = terrain->width;
		int height = terrain->height;

		for (int x = -width / 2; x < width / 2; x++)
		{
			glPushMatrix();
			glTranslatef((float)x, 0, 0);

			for (int z = -height / 2; z < height / 2; z++)
			{
				glPushMatrix();
				glTranslatef(0, 0, (float)z);

				glBegin(GL_TRIANGLE_FAN);

				float y0 = 0, y1 = 0, y2 = 0, y3 = 0;

				if (terrain->usePerlin)
				{
					y0 = get_terrain_height(terrain, (float)x, (float)z);
					y1 = get_terrain_height(terrain, (float)(x + 1), (float)z);
					y2 = get_terrain_height(terrain, (float)x, (float)(z + 1));
					y3 = get_terrain_height(terrain, (float)(x + 1), (float)(z + 1));
				}

				glColor4f(0.0, 1.0, 0.0, 1.0);
				glNormal3f(0, 1, 0);
				glTexCoord2f(0, 0);
				glVertex3f(0.0f, y0, 0.0f);
				glNormal3f(0, 1, 0);
				glTexCoord2f(1, 0);
				glVertex3f(1.0f, y1, 0.0f);
				glNormal3f(0, 1, 0);
				glTexCoord2f(1, 1);
				glVertex3f(1.0f, y3, 1.0f);
				glNormal3f(0, 1, 0);
				glTexCoord2f(1, 1);
				glVertex3f(1.0f, y3, 1.0f);
				glNormal3f(0, 1, 0);
				glTexCoord2f(0, 1);
				glVertex3f(0.0f, y2, 1.0f);
				glNormal3f(0, 1, 0);
				glTexCoord2f(0, 0);
				glVertex3f(0.0f, y0, 0.0f);

				glEnd();

				glPopMatrix();
			}

			glPopMatrix();
		}

		glEndList();
	}

	glCallList(*displayList);

	//glDisable(GL_TEXTURE_2D);
}

float get_terrain_height(terrain_t* terrain, float x, float z)
{
	return (float)((perlin((double)x / terrain->width, 0.5, (double)z / terrain->height) + 0.25) * 100);
}

GLfvector_t get_random_point_on_terrain(terrain_t* terrain)
{
	GLfvector_t ground_scale = get_scale(terrain->object.transform.m);

	float minX = -(terrain->width / 2) * ground_scale.x;
	float maxX = (terrain->width / 2) * ground_scale.x;

	float minZ = -(terrain->height / 2) * ground_scale.z;
	float maxZ = (terrain->height / 2) * ground_scale.z;

	float x = minX + ((maxX - minX) * ((float)rand() / (float)RAND_MAX));
	float z = minZ + ((maxZ - minZ) * (float)rand() / (float)RAND_MAX);
	float y = get_terrain_height(terrain, x / ground_scale.x, z / ground_scale.z) * ground_scale.y + 20.0f;

	return create_glfvector3(x, y, z);
}