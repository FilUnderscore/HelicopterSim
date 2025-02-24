#include "linkedlist.h"

#include <stdlib.h>

linkedlist_node_t* append_linkedlist_node(linkedlist_t* linked_list, void* value)
{
	linkedlist_node_t* node = (linkedlist_node_t*)calloc(1, sizeof(linkedlist_node_t));
	node->value = value;

	if (linked_list->start != 0)
	{
		linkedlist_node_t* current = linked_list->start;

		while (current->next != 0)
		{
			current = current->next;
		}

		current->next = node;
	}
	else
	{
		linked_list->start = node;
	}

	return node;
}