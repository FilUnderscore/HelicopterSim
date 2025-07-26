#include "building.h"

#include <GLFW/glfw3.h>
#include <stdlib.h>

object_t* create_building(GLfvector_t location, GLfvector_t scale)
{
	object_t* building = (object_t*)calloc(1, sizeof(object_t));

	building->transform = get_transform(
		create_glfvector3(location.x, location.y, location.z),
		create_glfvector3(0, 0, 0),
		create_glfvector3(scale.x, scale.y, scale.z)
	);

	//building->draw = draw_building;

	return building;
}

object_t* create_random_building(GLfvector_t location)
{
	float xScale = 20.0f + ((100.0f - 20.0f) * ((float)rand() / (float)RAND_MAX));
	float zScale = 20.0f + ((100.0f - 20.0f) * ((float)rand() / (float)RAND_MAX));
	float yScale = 50.0f + ((200.0f - 50.0f) * ((float)rand() / (float)RAND_MAX));

	return create_building(location, create_glfvector3(xScale, yScale, zScale));
}

void draw_building(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == -1)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		// TODO
		//glutSolidCube(1.0);

		glEndList();
	}

	glCallList(*displayList);
}