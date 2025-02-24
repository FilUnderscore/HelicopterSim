#pragma once

#include "scenegraph.h"

scenegraph_node_t* create_tree(GLfvector_t location);

void drawTreeLog(unsigned long* displayList, object_t* object);
void drawTreeLeaves(unsigned long* displayList, object_t* object);