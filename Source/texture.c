#include "texture.h"
#include "image.h"

#include <GLFW/glfw3.h>
#include <math.h>

#include <stdlib.h>

#define max(a, b) a > b ? a : b

texture_t* load_texture(const char* file)
{
	texture_t* texture = (texture_t*)calloc(1, sizeof(texture_t));
	loadImage(file, &texture->width, &texture->height, &texture->data);

	return texture;
}

void draw_texture(texture_t* tex)
{
	glEnable(GL_TEXTURE_2D);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex->width, tex->height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex->data);
	generate_texture_mipmap(tex);
}

void generate_texture_mipmap(texture_t* tex)
{
	/*
	int levels = (int)log2(max(tex->width, tex->height));
	int width = (int)(1 << (int)log2(tex->width) - 1);
	int height = (1 << (int)log2(tex->height) - 1);

	for (int level = 1; level < levels; level++)
	{
		glTexImage2D(GL_TEXTURE_2D, level, GL_RGB, tex->width, tex->height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex->data);
	}
	*/
}