#pragma once

typedef struct linkedlist_node
{
	struct linkedlist_node* next;
	void* value;
} linkedlist_node_t;

typedef struct linkedlist
{
	linkedlist_node_t* start;
} linkedlist_t;

linkedlist_node_t* append_linkedlist_node(linkedlist_t* linked_list, void* value);