#pragma once

#include <GLFW/glfw3.h>

#define KEY_MOVE_FORWARD	'w'
#define KEY_MOVE_BACKWARD	's'
#define KEY_MOVE_LEFT		'a'
#define KEY_MOVE_RIGHT		'd'
#define KEY_RENDER_FILL		'l'
#define KEY_CAMERA			'c'
#define KEY_FLOODLIGHT		'f'
#define KEY_EXIT			27 // Escape key.

// Define all GLUT special keys used for input (add any new key definitions here).

#define SP_KEY_MOVE_UP		GLFW_KEY_UP
#define SP_KEY_MOVE_DOWN	GLFW_KEY_DOWN
#define SP_KEY_TURN_LEFT	GLFW_KEY_LEFT
#define SP_KEY_TURN_RIGHT	GLFW_KEY_RIGHT

typedef enum inputtype
{
	key_pressed,
	key_released,
	mouse_moved
} inputtype_e;

typedef struct input
{
	inputtype_e type;
	void (*on_input)(struct input*);

	unsigned char key;
	int x;
	int y;
} input_t;

void bind_key_input(inputtype_e type, void (*on_input)(input_t*), unsigned char key);
void bind_mouse_input(inputtype_e type, void (*on_input)(input_t*));

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void keyPressed(unsigned char key, int x, int y);
void keyReleased(unsigned char key, int x, int y);
void mouseMoved(int x, int y);