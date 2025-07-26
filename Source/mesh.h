#pragma once

typedef struct mesh
{
	unsigned int vbo, vao, ebo, indexCount;
	//float* vertices;
} mesh_t;

mesh_t* create_mesh(float* vertices, unsigned int vertexCount, unsigned int* indices, unsigned int indexCount);
void draw_mesh(mesh_t* mesh);