#pragma once

#include "object.h"
#include "scenegraph.h"

scenegraph_node_t* create_turbine();

void drawWindTurbine(unsigned long* displayList);
void update_turbine(object_t* object, float dt);