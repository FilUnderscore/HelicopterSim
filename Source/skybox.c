#include "skybox.h"

/*
void drawSkybox(void)
{
	glDisable(GL_FOG);
	glDisable(GL_LIGHTING);
	glDepthMask(GL_FALSE);

	glEnable(GL_TEXTURE_2D);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	const float cubeVertices[][6][3] =
	{
		{
			{-1.0, -1.0, -0.5},
			{-1.0, 1.0, -0.5},
			{1.0, -1.0, -0.5},
			{1.0, -1.0, -0.5},
			{1.0, 1.0, -0.5},
			{-1.0, 1.0, -0.5},
		},
		{
			{-1.0, -1.0, 0.5},
			{-1.0, 1.0, 0.5},
			{-1.0, -1.0, -0.5},
			{-1.0, -1.0, -0.5},
			{-1.0, 1.0, -0.5},
			{-1.0, 1.0, 0.5},
		},
		{
			{1.0, -1.0, -0.5},
			{1.0, 1.0, -0.5},
			{1.0, -1.0, 0.5},
			{1.0, -1.0, 0.5},
			{1.0, 1.0, 0.5},
			{1.0, 1.0, -0.5},
		}
	};

	const float cubeTexCoords[][6][2] =
	{
		{
			{0, 0},
			{0, 1},
			{1, 0},
			{1, 0},
			{1, 1},
			{0, 1},
		},
		{
			{0, 0},
			{0, 1},
			{1, 0},
			{1, 0},
			{1, 1},
			{0, 1}
		},
		{
			{0, 0},
			{0, 1},
			{1, 0},
			{1, 0},
			{1, 1},
			{0, 1}
		}
	};

	for (int i = 0; i < 3; i++)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, 3, skyboxTextureWidths[i], skyboxTextureHeights[i], 0, GL_RGB, GL_UNSIGNED_BYTE, skyboxTextures[i]);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY_EXT);
		glTexCoordPointer(2, GL_FLOAT, 0, &cubeTexCoords[i]);
		glVertexPointer(3, GL_FLOAT, 0, &cubeVertices[i]);
		glDrawArrays(GL_TRIANGLES, 0, sizeof(cubeVertices[i]) / sizeof(cubeVertices[i][0]));
		glDisableClientState(GL_TEXTURE_COORD_ARRAY_EXT);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	glDisable(GL_TEXTURE_2D);

	glDepthMask(GL_TRUE);
	glEnable(GL_LIGHTING);
	glEnable(GL_FOG);
}
*/