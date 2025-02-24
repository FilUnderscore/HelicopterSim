#pragma once

#include "scenegraph.h"

object_t* create_building();
object_t* create_random_building();

void draw_building(unsigned long* displayList, object_t* object);