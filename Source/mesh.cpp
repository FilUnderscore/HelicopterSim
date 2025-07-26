#include "mesh.h"

#include <glad/glad.h>

#include <stdlib.h>

mesh_t* create_mesh(float* vertices, unsigned int vertexCount, unsigned int* indices, unsigned int indexCount)
{
	mesh_t* mesh = (mesh_t*)calloc(1, sizeof(mesh_t));

	glGenBuffers(1, &mesh->vbo);
	glGenBuffers(1, &mesh->ebo);

	glGenVertexArrays(1, &mesh->vao);
	glBindVertexArray(mesh->vao);

	glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo);
	glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(float), vertices, GL_STATIC_DRAW); // Copy mesh to GPU memory

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount, indices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexArrayAttrib(mesh->vao, 0);

	return mesh;
};

void draw_mesh(mesh_t* mesh)
{
	glBindVertexArray(mesh->vao);
	glDrawElements(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}