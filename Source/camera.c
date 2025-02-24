#include "camera.h"

#include "renderer.h"
#include "input.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "quaternion.h"

#include "matrix.h"

#include <GLFW/glfw3.h>
#include <gl/GL.h>

camera_t camera;
int dx, dy;

void on_camera_pre_process(void)
{
	GLfvector_t targetPosition = get_translation(camera.target->transform.m);
	//GLfvector_t targetRotation = get_rotation(camera.target->transform.m);
	GLfvector_t targetRotation = create_glfvector3(0, 0, 0);

	//GLfquaternion_t targetQuaternion = create_quaternion_axis_angle(0, 1, 0, targetRotation.);
	GLfquaternion_t targetQuaternion = get_rotation(camera.target->transform.m);
	GLfvector_t cameraRotPos = multiply_vector_by_quaternion(create_glfvector3(0, 0, -1), targetQuaternion);

	float camX = targetPosition.x + (cameraRotPos.x * 10.0f);
	float camZ = targetPosition.z + (-cameraRotPos.z * 10.0f);

	if (camera.orbitCamera)
	{
		// TODO
		/*
		gluLookAt(
			targetPosition.x + sin((camera.cameraYaw + targetRotation.y) * 3.1415f / 180.0f) * 10.0f,
			targetPosition.y + sin(camera.cameraPitch * 3.1415f / 180.0f) * 10.0f,
			targetPosition.z + cos((camera.cameraYaw + targetRotation.y) * 3.1415f / 180.0f) * 10.0f,
			targetPosition.x,
			targetPosition.y,
			targetPosition.z,
			0,
			1,
			0);
		*/

		DRAW_STRING((float)720 / 2.0f, 190, "Camera Yaw: %f Camera Pitch: %f", camera.cameraYaw, camera.cameraPitch);
	}
	else
	{
		float lookAtMatrix[16];

		lookat_matrix(lookAtMatrix, create_glfvector3(camX, targetPosition.y + 5.0f, camZ), create_glfvector3(targetPosition.x, targetPosition.y, targetPosition.z), create_glfvector3(0, 1, 0));
		glMultMatrixf(lookAtMatrix);
		glTranslatef(-camX, -(targetPosition.y + 5.0f), -camZ);

		printf("CAM %f %f\n", camX, camZ);
	}
}

void on_camera_key_pressed(input_t* input)
{
	camera.orbitCamera = !camera.orbitCamera;
	//glutSetCursor(camera.orbitCamera ? GLUT_CURSOR_NONE : GLUT_CURSOR_INHERIT);
}

void on_camera_mouse_moved(input_t* input)
{
	if (camera.orbitCamera == 0)
	{
		return;
	}

	int viewport_width = 720, viewport_height = 480;

	dx = (viewport_width / 2) - input->x;
	dy = (viewport_height / 2) - input->y;
}

void init_camera()
{
	bind_pre_processing(on_camera_pre_process);
	bind_key_input(key_pressed, on_camera_key_pressed, KEY_CAMERA);
	bind_mouse_input(mouse_moved, on_camera_mouse_moved);
}

void update_camera(float dt)
{
	if (!camera.orbitCamera)
	{
		return;
	}

	camera.cameraYaw = (float)fmod(camera.cameraYaw + dx * 50 * dt, 360.0f);
	camera.cameraPitch += dy * 50 * dt;

	if (camera.cameraPitch > 90.0f)
	{
		camera.cameraPitch = 90.0f;
	}
	else if (camera.cameraPitch < -90.0f)
	{
		camera.cameraPitch = -90.0f;
	}

	printf("%f %f %d %d\n", camera.cameraYaw, camera.cameraPitch, dx, dy);

	dx = 0;
	dy = 0;

}

void set_camera_target(object_t* target)
{
	camera.target = target;
}