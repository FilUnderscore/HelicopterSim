#include "transform.h"

#include <GLFW/glfw3.h>

#define _USE_MATH_DEFINES
#include <math.h>

GLfvector_t get_translation(float m[16])
{
	return create_glfvector3(m[12], m[13], m[14]);
}

GLfvector_t get_scale(float m[16])
{
	float x = sqrtf(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]);
	float y = sqrtf(m[1] * m[1] + m[5] * m[5] + m[9] * m[9]);
	float z = sqrtf(m[2] * m[2] + m[6] * m[6] + m[10] * m[10]);

	return create_glfvector3(x, y, z);
}

GLfquaternion_t get_rotation(float m[16])
{
	GLfquaternion_t quat;

	float m00 = m[0];
	float m11 = m[5];
	float m22 = m[10];

	float m21 = m[9];
	float m12 = m[6];
	float m02 = m[2];
	float m20 = m[8];
	float m10 = m[4];
	float m01 = m[1];

	float trace = m00 + m11 + m22 + 1.0f;

	if (trace > 0.00001f)
	{
		float s = 0.5f / sqrt(trace);

		quat.v.w = 0.25f / s;
		quat.v.x = (m21 - m12) * s;
		quat.v.y = (m02 - m20) * s;
		quat.v.z = (m10 - m01) * s;
	}
	else
	{
		if (m00 > m11 && m00 > m22)
		{
			float s = 2.0f * sqrt(1.0f + m00 - m11 - m22);

			quat.v.x = 0.25f * s;
			quat.v.y = (m01 + m10) / s;
			quat.v.z = (m02 + m20) / s;
			quat.v.w = (m12 - m21) / s;
		}
		else if (m11 > m22)
		{
			float s = 2.0f * sqrt(1.0f + m11 - m00 - m22);

			quat.v.x = (m01 + m10) / s;
			quat.v.y = 0.25f * s;
			quat.v.z = (m12 + m21) / s;
			quat.v.w = (m02 - m20) / s;
		}
		else
		{
			float s = 2.0f * sqrt(1.0f + m22 - m00 - m11);

			quat.v.x = (m02 + m20) / s;
			quat.v.y = (m12 + m21) / s;
			quat.v.z = 0.25f * s;
			quat.v.w = (m01 - m10) / s;
		}
	}

	const float mag = sqrtf(quat.v.x * quat.v.x + quat.v.y * quat.v.y + quat.v.z * quat.v.z + quat.v.w * quat.v.w);
	quat.v.x /= mag;
	quat.v.y /= mag;
	quat.v.z /= mag;
	quat.v.w /= mag;

	return quat;
}

transform_t get_transform(GLfvector_t translation, GLfvector_t rotation, GLfvector_t scale)
{
	transform_t transform = { 0 };

	glPushMatrix();
	glLoadIdentity();

	glTranslatef(translation.x, translation.y, translation.z);
	glScalef(scale.x, scale.y, scale.z);
	glRotatef(rotation.w, rotation.x, rotation.y, rotation.z);

	glGetFloatv(GL_MODELVIEW_MATRIX, transform.m);

	glPopMatrix();

	return transform;
}