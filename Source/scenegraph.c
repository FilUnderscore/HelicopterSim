#include "scenegraph.h"

#include <stdlib.h>

scenegraph_node_t* create_node(object_t* obj)
{
	scenegraph_node_t* node = calloc(1, sizeof(scenegraph_node_t));

	if (node != 0)
	{
		node->obj = obj;
		node->displayList = -1;
		node->children = 0;
		node->children_count = 0;
	}

	return node;
}

scenegraph_node_t* create_node_with_ptr(object_t obj)
{
	object_t* obj_ptr = (object_t*)calloc(1, sizeof(object_t));
	memcpy(obj_ptr, &obj, sizeof(object_t));

	return create_node(obj_ptr);
}

void append_child(scenegraph_node_t* parent, scenegraph_node_t* child)
{
	if (child == 0)
	{
		return;
	}

	scenegraph_node_t** children = parent->children;
	scenegraph_node_t** newChildren = malloc(sizeof(scenegraph_node_t*) * (parent->children_count + 1));

	if (newChildren == 0)
	{
		return;
	}

	if (parent->children != 0)
	{
		memcpy(newChildren, children, sizeof(scenegraph_node_t*) * parent->children_count);

		free(parent->children);
		parent->children = 0;
	}

	parent->children = newChildren;

	memcpy(&newChildren[parent->children_count], &child, sizeof(scenegraph_node_t*));

	parent->children_count += 1;
}

void for_each_node(scenegraph_node_t* node, void (*for_node_start)(scenegraph_node_t*), void (*for_node_end)(scenegraph_node_t*))
{
	if (node == 0)
	{
		return;
	}

	object_t* obj = node->obj;
	scenegraph_node_t** children = node->children;
	unsigned long long children_count = node->children_count;

	for_node_start(node);

	for (unsigned long long index = 0; index < children_count; index++)
	{
		scenegraph_node_t* child = children[index];
		for_each_node(child, for_node_start, for_node_end);
	}

	if (for_node_end != 0)
	{
		for_node_end(node);
	}
}