#include "turbine.h"
#include "helicopter.h"

/*
scenegraph_node_t* create_turbine()
{
	scenegraph_node_t* wind_turbine = create_node(
		create_object(
			get_transform(
				create_glfvector3(x, y, z),
				create_glfvector3(0, 0, 0),
				create_glfvector3(1, 1, 1)
			),
			NULL
		)
	);

	scenegraph_node_t* wind_turbine_pole = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector4(0, 1, 0, 90),
				create_glfvector3(20, 200, 20)
			),
			drawWindTurbine
		)
	);

	scenegraph_node_t* wind_turbine_rotor = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 190, 0),
				create_glfvector4(0, 0, 1, 90),
				create_glfvector3(50, 50, 50)
			),
			drawHelicopterRotor
		)
	);

	append_child(wind_turbine, wind_turbine_pole);
	append_child(wind_turbine, wind_turbine_rotor);
	
	return wind_turbine;
}

void update_turbine(object_t* object, float dt)
{
	glPushMatrix();
	glLoadMatrixf(object->transform.m);
	glRotatef(360.0f * (60.0f / 60.0f) * dt, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, object->transform.m);
	glPopMatrix();
}

void drawWindTurbine(GLuint* displayList)
{
	GLfloat diffuseMat[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == -1)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		glutSolidCylinder(1.0, 1.0, 20, 20);

		glEndList();
	}

	glCallList(*displayList);
}
*/