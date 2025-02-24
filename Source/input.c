#include "input.h"

#include "linkedlist.h"
#include <stdlib.h>

linkedlist_t input_list;

void on_input(inputtype_e type, unsigned char key, int x, int y)
{
	linkedlist_node_t* current = input_list.start;

	while (current != 0)
	{
		input_t* input = (input_t*)current->value;

		if (input->type != type)
		{
			current = current->next;
			continue;
		}

		switch (input->type)
		{
		case key_pressed:
		case key_released:
			if (input->key != key)
			{
				current = current->next;
				continue;
			}

			break;
		case mouse_moved:
			input->x = x;
			input->y = y;
			break;
		default:
			break;
		}

		input->on_input(input);
		current = current->next;
	}
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	switch (action)
	{
	case GLFW_PRESS:
		keyPressed(tolower(key), 0, 0);
		break;
	case GLFW_RELEASE:
		keyReleased(tolower(key), 0, 0);
		break;
	default:
		break;
	}
}

/*
	Called each time a character key (e.g. a letter, number, or symbol) is pressed.
*/
void keyPressed(unsigned char key, int x, int y)
{
	on_input(key_pressed, tolower(key), x, y);
}

/*
	Called each time a character key (e.g. a letter, number, or symbol) is released.
*/
void keyReleased(unsigned char key, int x, int y)
{
	on_input(key_released, tolower(key), x, y);
}

void mouseMoved(int x, int y)
{
	on_input(mouse_moved, 0, x, y);
}

void bind_key_input(inputtype_e type, void (*on_input)(input_t*), unsigned char key)
{
	input_t* input = (input_t*)calloc(1, sizeof(input_t));
	input->type = type;
	input->key = key;
	input->on_input = on_input;

	append_linkedlist_node(&input_list, input);
}

void bind_mouse_input(inputtype_e type, void (*on_input)(input_t*))
{
	input_t* input = (input_t*)calloc(1, sizeof(input_t));
	input->type = type;
	input->on_input = on_input;

	append_linkedlist_node(&input_list, input);
}