#include "scenegraph.h"

#include "terrain.h"
#include "helicopter.h"
#include "light.h"
#include "turbine.h"
#include "building.h"
#include "tree.h"
#include "camera.h"

#include <stdlib.h>

#define GRID_WIDTH 100
#define GRID_HEIGHT 100
#define GRID_SQUARE_SCALE 100
#define GRID_HEIGHT_SCALE 20

#define TREE_COUNT 1500 // Lower if performance is being affected.
#define BUILDING_COUNT 100
#define WIND_TURBINE_COUNT 20

scenegraph_node_t* scene_graph;

void init_game()
{
	// init_skybox();
	// init_camera();
	init_lights();

	scene_graph = (scenegraph_node_t*)calloc(1, sizeof(scenegraph_node_t));

	terrain_t* terrain = create_terrain(GRID_WIDTH, GRID_HEIGHT, load_texture("grass.ppm"));
	terrain->object.transform = get_transform(create_glfvector3(0, 0, 0), create_glfvector3(0, 0, 0), create_glfvector3(GRID_SQUARE_SCALE, GRID_HEIGHT_SCALE, GRID_SQUARE_SCALE));

	scenegraph_node_t* ground = create_node(terrain);
	scenegraph_node_t* helicopter = create_helicopter(terrain);

	append_child(scene_graph, ground);
	append_child(scene_graph, helicopter);

	for (int i = 0; i < TREE_COUNT; i++)
	{
		GLfvector_t location = get_random_point_on_terrain(terrain);

		if (location.y < 50.0f)
		{
			i--;
			continue;
		}

		append_child(scene_graph, create_tree(location));
	}

	for (int i = 0; i < BUILDING_COUNT; i++)
	{
		GLfvector_t location = get_random_point_on_terrain(terrain);

		if (location.y < 50.0f)
		{
			i--;
			continue;
		}

		append_child(scene_graph, create_node(create_random_building(location)));
	}

	/*
	for (int i = 0; i < sizeof(wind_turbine_rotors) / sizeof(wind_turbine_rotors[0]); i++)
	{
		GLfvector_t location = get_random_point_on_terrain(terrain);

		if (location.y < 700.0f)
		{
			i--;
			continue;
		}

		append_child(scene_graph, create_turbine());
	}
	*/

	light_t* sun_light = create_light();
	sun_light->enabled = 1;
	sun_light->color = create_glfvector3(1, 1, 1);
	sun_light->intensity = 1.0;
	sun_light->type = directional;
	sun_light->position = create_glfvector3(5, 5, 0);
	sun_light->dirty = 1;

	bind_post_processing(on_helicopter_post_process);
	bind_helicopter_input();
	init_camera();
	set_camera_target(helicopter->obj);

	set_scene_graph(scene_graph);
}

float _dt = 0.0f;

void update_node(scenegraph_node_t* node)
{
	if (node->obj == 0 || node->obj->update == 0)
	{
		return;
	}

	node->obj->update(node->obj, _dt);
}

void update_game(float dt)
{
	_dt = dt;
	update_camera(dt);
	for_each_node(scene_graph, update_node, 0);
}