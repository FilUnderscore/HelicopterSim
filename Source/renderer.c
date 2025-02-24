#include <GLFW/glfw3.h>

#include "renderer.h"
#include "linkedlist.h"

#include "matrix.h"

int dx = 0, dy = 0;
float cameraYaw = 0, cameraPitch = 0;
int orbitCamera = 0;

int viewport_width = 1000, viewport_height = 800;

// Render objects as filled polygons (1) or wireframes (0). Default filled.
int renderFillEnabled = 1;

scenegraph_node_t* scene;

linkedlist_t pre_processing_list, post_processing_list;

void render_game(float dt)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, renderFillEnabled ? GL_FILL : GL_LINE);

	glLoadIdentity();
	
	{
		linkedlist_node_t* current = pre_processing_list.start;

		while (current != 0)
		{
			void (*pre_process)(void) = current->value;
			pre_process();

			current = current->next;
		}
	}

	if (scene != 0)
	{
		draw_nodes(scene);
	}

	{
		linkedlist_node_t* current = post_processing_list.start;

		while (current != 0)
		{
			void (*post_process)(void) = current->value;
			post_process();

			current = current->next;
		}
	}
}

/*
	Called when the OpenGL window has been resized.
*/
void reshape(GLFWwindow* window, int width, int height)
{
	viewport_width = width;
	viewport_height = height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	float m1[16];
	perspective_matrix(m1, 89, (float)viewport_width / (float)viewport_height, 0.1f, 9000.0f);
	
	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, width, height);
}

void on_render_fill_key_pressed(input_t* input)
{
	renderFillEnabled = !renderFillEnabled;
}

/*
	Initialise OpenGL and set up our scene before we begin the render loop.
*/
void init_renderer(void)
{
	glClearColor(0.67f, 0.82f, 0.85f, 1.0f);
	glViewport(0, 0, 720, 480);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	float perspectiveMatrix[16];
	perspective_matrix(perspectiveMatrix, 89, (float)720 / (float)480, 0.01f, 9000.0f);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_FOG);

	GLfloat fogColor[4] = { 0.67f, 0.82f, 0.85f, 0.2f };
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogf(GL_FOG_MODE, GL_EXP);
	glFogf(GL_FOG_DENSITY, 0.0005f);

	bind_key_input(key_pressed, on_render_fill_key_pressed, KEY_RENDER_FILL);
}

void drawString(float x, float y, const unsigned char* string)
{
	/*
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, viewport_width, viewport_height, 0);

	glDisable(GL_LIGHTING);

	glColor4f(1.0, 1.0, 1.0, 1.0);
	glRasterPos2f(x + 3, y + 15); // Draw text in screen space.
	glutBitmapString(GLUT_BITMAP_8_BY_13, string);

	glEnable(GL_LIGHTING);

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	*/
}

void draw_node_start(scenegraph_node_t* node)
{
	glPushMatrix();

	if (node->obj != 0)
	{
		glMultMatrixf(node->obj->transform.m);

		if (node->obj->draw == 0)
		{
			return;
		}

		node->obj->draw(&node->displayList, node->obj);
	}
}

void draw_node_end(scenegraph_node_t* node)
{
	glPopMatrix();
}

void draw_nodes(scenegraph_node_t* node)
{
	for_each_node(node, draw_node_start, draw_node_end);
}

void bind_pre_processing(void (*pre_process)(void))
{
	append_linkedlist_node(&pre_processing_list, pre_process);
}

void bind_post_processing(void (*post_process)(void))
{
	append_linkedlist_node(&post_processing_list, post_process);
}

void set_scene_graph(scenegraph_node_t* scene_graph)
{
	scene = scene_graph;
}