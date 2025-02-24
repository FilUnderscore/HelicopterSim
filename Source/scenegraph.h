#pragma once

#include "object.h"

typedef struct scenegraph_node
{
	object_t* obj;
	unsigned int displayList;

	struct scenegraph_node** children;
	unsigned long long children_count;
} scenegraph_node_t;

scenegraph_node_t* create_node(object_t* obj);
scenegraph_node_t* create_node_with_ptr(object_t obj);

void append_child(scenegraph_node_t* parent, scenegraph_node_t* child);

void for_each_node(scenegraph_node_t* node, void (*for_node_start)(scenegraph_node_t*), void (*for_node_end)(scenegraph_node_t*));