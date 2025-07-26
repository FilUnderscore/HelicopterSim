#pragma once

typedef struct texture
{
	unsigned char* data;
	unsigned int width, height;
	unsigned int gid;
} texture_t;

texture_t* load_texture(const char* file);
void draw_texture(texture_t* tex);
void generate_texture_mipmap(texture_t* tex);