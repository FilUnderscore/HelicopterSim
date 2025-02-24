#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "OpenGL32.lib")

#include <Windows.h>
#include <GLFW/glfw3.h>
#include <math.h>
#include <stdio.h>

#include "input.h"
#include "game.h"
#include "renderer.h"

#include "matrix.h"

int main(int argc, char **argv)
{
	GLFWwindow* window;

	if (!glfwInit())
	{
		return -1;
	}

	window = glfwCreateWindow(720, 480, "Helicopter Simulator", NULL, NULL);

	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glfwSetKeyCallback(window, key_callback);
	glfwSetWindowSizeCallback(window, reshape);

	init_renderer();
	init_game();

	double previousTime = glfwGetTime();
	double deltaTime = 0;

	while (!glfwWindowShouldClose(window))
	{
		double time = glfwGetTime();
		deltaTime += (time - previousTime) / (1.0f / 60.0f);
		previousTime = time;

		while (deltaTime >= 1.0)
		{
			update_game(1.0f / 60.0f);
			deltaTime--;
		}

		render_game(deltaTime);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}