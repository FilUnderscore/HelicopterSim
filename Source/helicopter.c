#include "helicopter.h"

#include "vector.h"
#include "transform.h"
#include "object.h"

#include "input.h"

#include "scenegraph.h"

#include "terrain.h"
#include "renderer.h"

#include <GLFW/glfw3.h>
#include <math.h>

#include <stdlib.h>

#define MOTION_NONE 0				// No motion.
#define MOTION_CLOCKWISE -1			// Clockwise rotation.
#define MOTION_ANTICLOCKWISE 1		// Anticlockwise rotation.
#define MOTION_BACKWARD -1			// Backward motion.
#define MOTION_FORWARD 1			// Forward motion.
#define MOTION_LEFT -1				// Leftward motion.
#define MOTION_RIGHT 1				// Rightward motion.
#define MOTION_DOWN -1				// Downward motion.
#define MOTION_UP 1					// Upward motion.

// Represents the motion of an object on four axes (Yaw, Surge, Sway, and Heave).
// 
// You can use any numeric values, as specified in the comments for each axis. However,
// the MOTION_ definitions offer an easy way to define a "unit" movement without using
// magic numbers (e.g. instead of setting Surge = 1, you can set Surge = MOTION_FORWARD).
//
typedef struct {
	int Yaw;		// Turn about the Z axis	[<0 = Clockwise, 0 = Stop, >0 = Anticlockwise]
	int Surge;		// Move forward or back		[<0 = Backward,	0 = Stop, >0 = Forward]
	int Sway;		// Move sideways (strafe)	[<0 = Left, 0 = Stop, >0 = Right]
	int Heave;		// Move vertically			[<0 = Down, 0 = Stop, >0 = Up]
} motionstate4_t;

/******************************************************************************
 * Keyboard Input Handling Setup
 ******************************************************************************/

 // Represents the state of a single keyboard key.Represents the state of a single keyboard key.
typedef enum {
	KEYSTATE_UP = 0,	// Key is not pressed.
	KEYSTATE_DOWN		// Key is pressed down.
} keystate_t;

// Represents the states of a set of keys used to control an object's motion.
typedef struct {
	keystate_t MoveForward;
	keystate_t MoveBackward;
	keystate_t MoveLeft;
	keystate_t MoveRight;
	keystate_t MoveUp;
	keystate_t MoveDown;
	keystate_t TurnLeft;
	keystate_t TurnRight;
} motionkeys_t;

// Current state of all keys used to control our "player-controlled" object's motion.
motionkeys_t motionKeyStates = {
	KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP,
	KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP };

// How our "player-controlled" object should currently be moving, solely based on keyboard input.
//
// Note: this may not represent the actual motion of our object, which could be subject to
// other controls (e.g. mouse input) or other simulated forces (e.g. gravity).
motionstate4_t keyboardMotion = { MOTION_NONE, MOTION_NONE, MOTION_NONE, MOTION_NONE };

// Define all character keys used for input (add any new key definitions here).
// Note: USE ONLY LOWERCASE CHARACTERS HERE. The keyboard handler provided converts all
// characters typed by the user to lowercase, so the SHIFT key is ignored.

helicopter_t* helicopter = NULL;

void drawHelicopterBody(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 1.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	// TODO
	//glutSolidSphere(1.0, 20, 20);
}

void drawHelicopterTail(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 1.0, 1.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	// TODO
	//glutSolidCylinder(1.0, 1.0, 20, 20);
}

void drawHelicopterRotor(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 1.0, 1.0, 1.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);

	glPushMatrix();
	glTranslatef(0, 0.1f, 0);
	glRotatef(90, 1, 0, 0);
	// TODO
	//glutSolidCylinder(0.1, 0.1, 10, 10);
	glPopMatrix();

	float vertices[][3] =
	{
		{ -1.0f, 0.105f, -0.05f },
		{ 1.0f, 0.105f, -0.05f },
		{ 1.0f, 0.105f, 0.05f },

		{ 1.0f, 0.105f, 0.05f },
		{ -1.0f, 0.105f, 0.05f },
		{ -1.0f, 0.105f, -0.05f },

		{ -0.05f, 0.105f, -1.0f },
		{ 0.05f, 0.105f, -1.0f },
		{ 0.05f, 0.105f, 1.0f },

		{ 0.05f, 0.105f, 1.0f },
		{ -0.05f, 0.105f, 1.0f },
		{ -0.05f, 0.105f, -1.0f },
	};

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &vertices);
	glDrawArrays(GL_TRIANGLES, 0, sizeof(vertices) / sizeof(vertices[0]));
	glDisableClientState(GL_VERTEX_ARRAY);
}

void drawHelicopterLeg(GLuint* displayList, object_t* object)
{
	GLfloat diffuseMat[] = { 0.0, 0.0, 1.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);

	// TODO
	//glutSolidCylinder(0.2, 0.5, 10, 10);
}

void update_helicopter(object_t* object, float dt)
{
	helicopter_t* helicopter = (helicopter_t*)object;

	// Update helicopter altitude
	GLfvector_t helicopter_position = get_translation(object->transform.m);

	GLfvector_t ground_scale = get_scale(helicopter->ground->transform.m);
	float ground_height = get_terrain_height((terrain_t*)helicopter->ground, helicopter_position.x / ground_scale.x, helicopter_position.z / ground_scale.z) * ground_scale.y + 1.0f;

	if (ground_height < 0.0f)
	{
		ground_height = 0.0f;
	}

	float altitude = ((helicopter->helicopterRotorSpeed * helicopter->helicopterRotorSpeed) / 1000.0f) - 20.0f;

	if (altitude < 0.0f)
	{
		altitude = 0.0f;
	}

	{
		if (altitude >= 0.0f)
		{
			glPushMatrix();
			glLoadMatrixf(object->transform.m);
			glTranslatef(0, -helicopter_position.y, 0);
			glTranslatef(0, ground_height + altitude, 0);
			glGetFloatv(GL_MODELVIEW_MATRIX, object->transform.m);
			glPopMatrix();
		}

		helicopter->helicopterRotorSpeed -= 5.0f * dt;

		if (altitude <= 0.0f)
		{
			helicopter->helicopterRotorSpeed -= 40.0f * dt;
		}

		if (helicopter->helicopterRotorSpeed < 0.0f)
		{
			helicopter->helicopterRotorSpeed = 0.0f;
		}
	}

	// Update rotors
	glPushMatrix();
	glLoadMatrixf(helicopter->helicopter_rotor_shaft->transform.m);
	glRotatef(360.0f * (helicopter->helicopterRotorSpeed / 60.0f) * dt, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, helicopter->helicopter_rotor_shaft->transform.m);
	glPopMatrix();

	glPushMatrix();
	glLoadMatrixf(helicopter->helicopter_tail_rotor_shaft->transform.m);
	glRotatef(4.0f * 360.0f * (helicopter->helicopterRotorSpeed / 60.0f) * dt, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, helicopter->helicopter_tail_rotor_shaft->transform.m);
	glPopMatrix();

	// Update flood light
	light_t* flood_light = helicopter->flood_light;

	flood_light->position = helicopter_position;

	/*
		Keyboard motion handler: complete this section to make your "player-controlled"
		object respond to keyboard input.
	*/
	if (keyboardMotion.Yaw != MOTION_NONE) {
		/* TEMPLATE: Turn your object right (clockwise) if .Yaw < 0, or left (anticlockwise) if .Yaw > 0 */

		GLfvector_t helicopter_position = get_translation(object->transform.m);

		if (helicopter_position.y > ground_height)
		{
			glPushMatrix();
			glLoadMatrixf(object->transform.m);

			glRotatef(dt * 100 * keyboardMotion.Yaw, 0, 1, 0);
			helicopter->helicopterHeading = (float)fmod(helicopter->helicopterHeading + dt * 100 * keyboardMotion.Yaw, 360.0f);

			if (helicopter->helicopterHeading < 0.0f)
			{
				helicopter->helicopterHeading += 360.0f;
			}

			glGetFloatv(GL_MODELVIEW_MATRIX, object->transform.m);

			glPopMatrix();
		}
	}
	if (keyboardMotion.Surge != MOTION_NONE) {
		/* TEMPLATE: Move your object backward if .Surge < 0, or forward if .Surge > 0 */

		GLfvector_t helicopter_position = get_translation(object->transform.m);

		if (helicopter_position.y > ground_height)
		{
			glPushMatrix();
			glLoadMatrixf(object->transform.m);
			glTranslatef(0, 0, -keyboardMotion.Surge * dt * 66);
			glGetFloatv(GL_MODELVIEW_MATRIX, object->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Sway != MOTION_NONE) {
		/* TEMPLATE: Move (strafe) your object left if .Sway < 0, or right if .Sway > 0 */
		GLfvector_t helicopter_position = get_translation(object->transform.m);

		if (helicopter_position.y > ground_height)
		{
			glPushMatrix();
			glLoadMatrixf(object->transform.m);
			glTranslatef(keyboardMotion.Sway * dt * 66, 0, 0);
			glGetFloatv(GL_MODELVIEW_MATRIX, object->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Heave != MOTION_NONE) {
		/* TEMPLATE: Move your object down if .Heave < 0, or up if .Heave > 0 */
		helicopter->helicopterRotorSpeed += keyboardMotion.Heave * 80.0f * dt;
	}
}

scenegraph_node_t* create_helicopter(object_t* ground)
{
	helicopter_t* helicopter_obj = (helicopter_t*)calloc(1, sizeof(helicopter_t));
	helicopter_obj->ground = ground;
	helicopter = helicopter_obj;

	helicopter_obj->object.transform = get_transform(
		create_glfvector3(0, 0, 0),
		create_glfvector4(0, 1, 0, 0),
		create_glfvector3(1, 1, 1)
	);

	helicopter_obj->object.update = update_helicopter;

	scenegraph_node_t* helicopter_node = create_node(helicopter_obj);

	scenegraph_node_t* helicopter_body = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(1, 1, 2)
			),
			drawHelicopterBody,
			NULL
		)
	);

	scenegraph_node_t* helicopter_tail = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0, 0.0f, 0.5f),
				create_glfvector3(0, 0, 0),
				create_glfvector3(0.125f, 0.25f, 1.5f)
			),
			drawHelicopterTail,
			NULL
		)
	);

	scenegraph_node_t* helicopter_rotor = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0, 1, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(2, 1, 2)
			),
			drawHelicopterRotor,
			NULL
		)
	);

	scenegraph_node_t* helicopter_tail_rotor = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(-1.0f, 0.0f, 0.95f),
				create_glfvector4(0, 0, 1, 90),
				create_glfvector3(2, 2, 0.185f)
			),
			drawHelicopterRotor,
			NULL
		)
	);

	scenegraph_node_t* helicopter_leg_l = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(-0.5f, -0.5f, 0.0f),
				create_glfvector4(1, 0, 0, 90),
				create_glfvector3(1, 1, 3)
			),
			drawHelicopterLeg,
			NULL
		)
	);

	scenegraph_node_t* helicopter_leg_r = create_node_with_ptr(
		create_object(
			get_transform(
				create_glfvector3(0.5f, -0.5f, 0.0f),
				create_glfvector4(1, 0, 0, 90),
				create_glfvector3(1, 1, 3)
			),
			drawHelicopterLeg,
			NULL
		)
	);

	append_child(helicopter_node, helicopter_body);
	append_child(helicopter_body, helicopter_tail);
	append_child(helicopter_tail, helicopter_tail_rotor);
	append_child(helicopter_node, helicopter_rotor);
	append_child(helicopter_body, helicopter_leg_l);
	append_child(helicopter_body, helicopter_leg_r);

	light_t* flood_light = create_light();
	flood_light->enabled = 1;
	flood_light->color = create_glfvector3(1, 1, 1);
	flood_light->intensity = 100.0;
	flood_light->type = spot;
	GLfvector_t floodLightDir = create_glfvector3(0, -1, 0);
	flood_light->spotDirection = floodLightDir;
	flood_light->spotCutoffAngle = 30;
	flood_light->position = get_translation(helicopter_obj->object.transform.m);
	flood_light->dirty = 1;

	helicopter_obj->flood_light = flood_light;
	helicopter_obj->helicopter_rotor_shaft = helicopter_rotor->obj;
	helicopter_obj->helicopter_tail_rotor_shaft = helicopter_tail_rotor->obj;

	return helicopter_node;
}

void on_key_pressed_move_forward(input_t* input)
{
	motionKeyStates.MoveForward = KEYSTATE_DOWN;
	keyboardMotion.Surge = MOTION_FORWARD;
}

void on_key_pressed_move_backward(input_t* input)
{
	motionKeyStates.MoveBackward = KEYSTATE_DOWN;
	keyboardMotion.Surge = MOTION_BACKWARD;
}

void on_key_pressed_move_left(input_t* input)
{
	motionKeyStates.MoveLeft = KEYSTATE_DOWN;
	keyboardMotion.Sway = MOTION_LEFT;
}

void on_key_pressed_move_right(input_t* input)
{
	motionKeyStates.MoveRight = KEYSTATE_DOWN;
	keyboardMotion.Sway = MOTION_RIGHT;
}

void on_key_pressed_floodlight(input_t* input)
{
	light_t* flood_light = helicopter->flood_light;
	flood_light->enabled = !flood_light->enabled;
	flood_light->dirty = 1;
}

void on_key_released_move_forward(input_t* input)
{
	motionKeyStates.MoveForward = KEYSTATE_UP;
	keyboardMotion.Surge = (motionKeyStates.MoveBackward == KEYSTATE_DOWN) ? MOTION_BACKWARD : MOTION_NONE;
}

void on_key_released_move_backward(input_t* input)
{
	motionKeyStates.MoveBackward = KEYSTATE_UP;
	keyboardMotion.Surge = (motionKeyStates.MoveForward == KEYSTATE_DOWN) ? MOTION_FORWARD : MOTION_NONE;
}

void on_key_released_move_left(input_t* input)
{
	motionKeyStates.MoveLeft = KEYSTATE_UP;
	keyboardMotion.Sway = (motionKeyStates.MoveRight == KEYSTATE_DOWN) ? MOTION_RIGHT : MOTION_NONE;
}

void on_key_released_move_right(input_t* input)
{
	motionKeyStates.MoveRight = KEYSTATE_UP;
	keyboardMotion.Sway = (motionKeyStates.MoveLeft == KEYSTATE_DOWN) ? MOTION_LEFT : MOTION_NONE;
}

void on_special_key_pressed_move_up(input_t* input)
{
	motionKeyStates.MoveUp = KEYSTATE_DOWN;
	keyboardMotion.Heave = MOTION_UP;
}

void on_special_key_pressed_move_down(input_t* input)
{
	motionKeyStates.MoveDown = KEYSTATE_DOWN;
	keyboardMotion.Heave = MOTION_DOWN;
}

void on_special_key_pressed_turn_left(input_t* input)
{
	motionKeyStates.TurnLeft = KEYSTATE_DOWN;
	keyboardMotion.Yaw = MOTION_ANTICLOCKWISE;
}

void on_special_key_pressed_turn_right(input_t* input)
{
	motionKeyStates.TurnRight = KEYSTATE_DOWN;
	keyboardMotion.Yaw = MOTION_CLOCKWISE;
}

void on_special_key_released_move_up(input_t* input)
{
	motionKeyStates.MoveUp = KEYSTATE_UP;
	keyboardMotion.Heave = (motionKeyStates.MoveDown == KEYSTATE_DOWN) ? MOTION_DOWN : MOTION_NONE;
}

void on_special_key_released_move_down(input_t* input)
{
	motionKeyStates.MoveDown = KEYSTATE_UP;
	keyboardMotion.Heave = (motionKeyStates.MoveUp == KEYSTATE_DOWN) ? MOTION_UP : MOTION_NONE;
}

void on_special_key_released_turn_left(input_t* input)
{
	motionKeyStates.TurnLeft = KEYSTATE_UP;
	keyboardMotion.Yaw = (motionKeyStates.TurnRight == KEYSTATE_DOWN) ? MOTION_CLOCKWISE : MOTION_NONE;
}

void on_special_key_released_turn_right(input_t* input)
{
	motionKeyStates.TurnRight = KEYSTATE_UP;
	keyboardMotion.Yaw = (motionKeyStates.TurnLeft == KEYSTATE_DOWN) ? MOTION_ANTICLOCKWISE : MOTION_NONE;
}

void bind_helicopter_input()
{
	bind_key_input(key_pressed, on_key_pressed_move_forward, KEY_MOVE_FORWARD);
	bind_key_input(key_pressed, on_key_pressed_move_backward, KEY_MOVE_BACKWARD);
	bind_key_input(key_pressed, on_key_pressed_move_left, KEY_MOVE_LEFT);
	bind_key_input(key_pressed, on_key_pressed_move_right, KEY_MOVE_RIGHT);
	bind_key_input(key_pressed, on_key_pressed_floodlight, KEY_FLOODLIGHT);
	bind_key_input(key_released, on_key_released_move_forward, KEY_MOVE_FORWARD);
	bind_key_input(key_released, on_key_released_move_backward, KEY_MOVE_BACKWARD);
	bind_key_input(key_released, on_key_released_move_left, KEY_MOVE_LEFT);
	bind_key_input(key_released, on_key_released_move_right, KEY_MOVE_RIGHT);
	bind_key_input(key_pressed, on_special_key_pressed_move_up, SP_KEY_MOVE_UP);
	bind_key_input(key_pressed, on_special_key_pressed_move_down, SP_KEY_MOVE_DOWN);
	bind_key_input(key_pressed, on_special_key_pressed_turn_left, SP_KEY_TURN_LEFT);
	bind_key_input(key_pressed, on_special_key_pressed_turn_right, SP_KEY_TURN_RIGHT);
	bind_key_input(key_released, on_special_key_released_move_up, SP_KEY_MOVE_UP);
	bind_key_input(key_released, on_special_key_released_move_down, SP_KEY_MOVE_DOWN);
	bind_key_input(key_released, on_special_key_released_turn_left, SP_KEY_TURN_LEFT);
	bind_key_input(key_released, on_special_key_released_turn_right, SP_KEY_TURN_RIGHT);
}

void on_helicopter_post_process()
{
	if (helicopter == 0)
	{
		return;
	}

	DRAW_STRING(0, 0, "Helicopter Position");

	GLfvector_t helicopter_position = get_translation(helicopter->object.transform.m);
	DRAW_STRING(0, 15, "x: %f y: %f z: %f", helicopter_position.x, helicopter_position.y, helicopter_position.z);

	DRAW_STRING(0, 30, "Helicopter Heading");
	DRAW_STRING(0, 45, "%.2f degrees", helicopter->helicopterHeading);

	DRAW_STRING(0, 60, "Helicopter Rotor Speed:");
	DRAW_STRING(0, 75, "%f RPM", helicopter->helicopterRotorSpeed);

	DRAW_STRING(0, 100, "Controls:");
	DRAW_STRING(0, 115, "UP/DOWN ARROWS: Increase (move up) or decrease (move down) altitude.");
	DRAW_STRING(0, 130, "LEFT/RIGHT ARROWS: Turn left/right.");
	DRAW_STRING(0, 145, "W/S: Move forward/backward.");
	DRAW_STRING(0, 160, "A/D: Strafe left/right.");
	DRAW_STRING(0, 175, "L: Toggle wireframe mode.");
	DRAW_STRING(0, 190, "C: Toggle camera mode.");
	DRAW_STRING(0, 205, "F: Toggle helicopter flood light.");
}