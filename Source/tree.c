#include "tree.h"

#include <GLFW/glfw3.h>

scenegraph_node_t* create_tree(GLfvector_t location)
{
	scenegraph_node_t* tree = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(location.x, location.y, location.z),
				create_glfvector3(0, 0, 0),
				create_glfvector3(1, 1, 1)
			),
			NULL,
			NULL
		)
	);

	scenegraph_node_t* tree_log = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector4(1, 0, 0, 90),
				create_glfvector3(1, 2, 1)
			),
			drawTreeLog,
			NULL
		)
	);

	scenegraph_node_t* tree_leaves = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0, 30, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(20, 40, 20)
			),
			drawTreeLeaves,
			NULL
		)
	);

	append_child(tree, tree_log);
	append_child(tree, tree_leaves);

	return tree;
}

void drawTreeLog(unsigned long* displayList, object_t* object)
{
	float diffuseMat[] = { 0.38f, 0.23f, 0.08f, 1.0f };
	float ambientMat[] = { 0.0f, 0.0f, 0.0f, 1.0f };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == -1)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		// TODO
		//glutSolidCylinder(10.0, 20.0, 10, 10);

		glEndList();
	}

	glCallList(*displayList);
}

void drawTreeLeaves(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 0.0, 1.0, 0.0, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == -1)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		// TODO
		//glutSolidSphere(1.0, 10, 10);

		glEndList();
	}

	glCallList(*displayList);
}