#pragma once

#include "scenegraph.h"
#include "input.h"
#include <stdio.h>

#include <GLFW/glfw3.h>

#define DRAW_STRING(x, y, ...) { unsigned char str[256]; snprintf(str, sizeof str, __VA_ARGS__); drawString(x, y, str); }

void render_game(float dt);
void reshape(GLFWwindow* window, int width, int height);
void idle(void);
void init_renderer(void);

void drawString(float x, float y, const unsigned char* string);

void bind_pre_processing(void (*pre_process)(void));
void bind_post_processing(void (*post_process)(void));
void set_scene_graph(scenegraph_node_t* scene_graph);
void draw_nodes(scenegraph_node_t* node);