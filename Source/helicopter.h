#pragma once

#include "object.h"
#include "light.h"

#include "scenegraph.h"

typedef struct
{
	object_t object;
	float helicopterHeading;
	float helicopterPitch;
	float helicopterRotorSpeed;

	object_t* helicopter_rotor_shaft;
	object_t* helicopter_tail_rotor_shaft;
	light_t* flood_light;
	object_t* ground;
} helicopter_t;

void drawHelicopterBody(unsigned long* displayList, object_t* object);
void drawHelicopterTail(unsigned long* displayList, object_t* object);
void drawHelicopterRotor(unsigned long* displayList, object_t* object);
void drawHelicopterLeg(unsigned long* displayList, object_t* object);

void update_helicopter(object_t* object, float dt);

scenegraph_node_t* create_helicopter(object_t* ground);
void bind_helicopter_input();

void on_helicopter_post_process();