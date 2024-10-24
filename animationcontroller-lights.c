/******************************************************************************
 *
 * Computer Graphics Programming 2020 Project Template v1.0 (11/04/2021)
 *
 * Based on: Animation Controller v1.0 (11/04/2021)
 *
 * This template provides a basic FPS-limited render loop for an animated scene,
 * plus keyboard handling for smooth game-like control of an object such as a
 * character or vehicle.
 *
 * A simple static lighting setup is provided via initLights(), which is not
 * included in the animationalcontrol.c template. There are no other changes.
 *
 ******************************************************************************/

#include <Windows.h>
#include <freeglut.h>
#include <math.h>
#include <stdio.h>

 /******************************************************************************
  * Animation & Timing Setup
  ******************************************************************************/

  // Target frame rate (number of Frames Per Second).
#define TARGET_FPS 60				

// Ideal time each frame should be displayed for (in milliseconds).
const unsigned int FRAME_TIME = 1000 / TARGET_FPS;

// Frame time in fractional seconds.
// Note: This is calculated to accurately reflect the truncated integer value of
// FRAME_TIME, which is used for timing, rather than the more accurate fractional
// value we'd get if we simply calculated "FRAME_TIME_SEC = 1.0f / TARGET_FPS".
const float FRAME_TIME_SEC = (1000 / TARGET_FPS) / 1000.0f;

// Time we started preparing the current frame (in milliseconds since GLUT was initialized).
unsigned int frameStartTime = 0;

/******************************************************************************
 * Some Simple Definitions of Motion
 ******************************************************************************/

#define MOTION_NONE 0				// No motion.
#define MOTION_CLOCKWISE -1			// Clockwise rotation.
#define MOTION_ANTICLOCKWISE 1		// Anticlockwise rotation.
#define MOTION_BACKWARD -1			// Backward motion.
#define MOTION_FORWARD 1			// Forward motion.
#define MOTION_LEFT -1				// Leftward motion.
#define MOTION_RIGHT 1				// Rightward motion.
#define MOTION_DOWN -1				// Downward motion.
#define MOTION_UP 1					// Upward motion.

 // Represents the motion of an object on four axes (Yaw, Surge, Sway, and Heave).
 // 
 // You can use any numeric values, as specified in the comments for each axis. However,
 // the MOTION_ definitions offer an easy way to define a "unit" movement without using
 // magic numbers (e.g. instead of setting Surge = 1, you can set Surge = MOTION_FORWARD).
 //
typedef struct {
	int Yaw;		// Turn about the Z axis	[<0 = Clockwise, 0 = Stop, >0 = Anticlockwise]
	int Surge;		// Move forward or back		[<0 = Backward,	0 = Stop, >0 = Forward]
	int Sway;		// Move sideways (strafe)	[<0 = Left, 0 = Stop, >0 = Right]
	int Heave;		// Move vertically			[<0 = Down, 0 = Stop, >0 = Up]
} motionstate4_t;

/******************************************************************************
 * Keyboard Input Handling Setup
 ******************************************************************************/

// Represents the state of a single keyboard key.Represents the state of a single keyboard key.
typedef enum {
	KEYSTATE_UP = 0,	// Key is not pressed.
	KEYSTATE_DOWN		// Key is pressed down.
} keystate_t;

// Represents the states of a set of keys used to control an object's motion.
typedef struct {
	keystate_t MoveForward;
	keystate_t MoveBackward;
	keystate_t MoveLeft;
	keystate_t MoveRight;
	keystate_t MoveUp;
	keystate_t MoveDown;
	keystate_t TurnLeft;
	keystate_t TurnRight;
} motionkeys_t;

// Current state of all keys used to control our "player-controlled" object's motion.
motionkeys_t motionKeyStates = {
	KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP,
	KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP, KEYSTATE_UP };

// How our "player-controlled" object should currently be moving, solely based on keyboard input.
//
// Note: this may not represent the actual motion of our object, which could be subject to
// other controls (e.g. mouse input) or other simulated forces (e.g. gravity).
motionstate4_t keyboardMotion = { MOTION_NONE, MOTION_NONE, MOTION_NONE, MOTION_NONE };

// Define all character keys used for input (add any new key definitions here).
// Note: USE ONLY LOWERCASE CHARACTERS HERE. The keyboard handler provided converts all
// characters typed by the user to lowercase, so the SHIFT key is ignored.

#define KEY_MOVE_FORWARD	'w'
#define KEY_MOVE_BACKWARD	's'
#define KEY_MOVE_LEFT		'a'
#define KEY_MOVE_RIGHT		'd'
#define KEY_RENDER_FILL		'l'
#define KEY_CAMERA			'c'
#define KEY_EXIT			27 // Escape key.

// Define all GLUT special keys used for input (add any new key definitions here).

#define SP_KEY_MOVE_UP		GLUT_KEY_UP
#define SP_KEY_MOVE_DOWN	GLUT_KEY_DOWN
#define SP_KEY_TURN_LEFT	GLUT_KEY_LEFT
#define SP_KEY_TURN_RIGHT	GLUT_KEY_RIGHT

/******************************************************************************
 * GLUT Callback Prototypes
 ******************************************************************************/

void display(void);
void reshape(int width, int height);
void keyPressed(unsigned char key, int x, int y);
void specialKeyPressed(int key, int x, int y);
void keyReleased(unsigned char key, int x, int y);
void specialKeyReleased(int key, int x, int y);
void mouseMoved(int x, int y);
void idle(void);

/******************************************************************************
 * Animation-Specific Function Prototypes (add your own here)
 ******************************************************************************/

void main(int argc, char **argv);
void init(void);
void think(void);
void initLights(void);

void initScene(void);

void drawScene(void);
void drawNode(struct scenegraph_node_t* node);

void drawGrid(void);
void drawHelicopterBody(void);
void drawHelicopterTail(void);
void drawHelicopterRotorShaft(void);
void drawHelicopterRotor(void);

void drawString(float x, float y, const unsigned char* string);

/******************************************************************************
 * Animation-Specific Setup (Add your own definitions, constants, and globals here)
 ******************************************************************************/

// Orbit camera settings
int dx = 0, dy = 0;
float cameraYaw = 0, cameraPitch = 0;
int orbitCamera = 0;

int viewport_width = 1000, viewport_height = 800;

float helicopterHeading = 0;
float helicopterPitch = 0;
float helicopterRotorSpeed = 0;

// Render objects as filled polygons (1) or wireframes (0). Default filled.
int renderFillEnabled = 1;

#define GRID_WIDTH 100
#define GRID_HEIGHT 100

typedef struct
{
	GLfloat x;
	GLfloat y;
	GLfloat z;
	GLfloat w;
} GLfvector_t;

typedef struct
{
	GLfloat m[16];
} transform_t;

typedef struct
{
	transform_t transform;
	void (*draw)();
} object_t;

typedef struct
{
	object_t* obj;

	struct scenegraph_node_t** children;
	size_t children_count;
} scenegraph_node_t;

scenegraph_node_t* scene_graph = NULL;
object_t* _helicopter = NULL;
object_t* _helicopter_rotor_shaft = NULL;
object_t* _helicopter_tail_rotor_shaft = NULL;

transform_t get_transform(GLfvector_t translation, GLfvector_t rotation, GLfvector_t scale);
object_t* create_object(transform_t transform, void (*draw)());
GLfvector_t create_glfvector4(GLfloat x, GLfloat y, GLfloat z, GLfloat w);
GLfvector_t create_glfvector3(GLfloat x, GLfloat y, GLfloat z);
scenegraph_node_t* create_node(object_t* obj);
void append_child(scenegraph_node_t* parent, scenegraph_node_t* child);

GLfvector_t get_translation(GLfloat m[16]);

#define DRAW_STRING(x, y, ...) { unsigned char str[256]; snprintf(str, sizeof str, __VA_ARGS__); drawString(x, y, str); }

/******************************************************************************
 * Entry Point (don't put anything except the main function here)
 ******************************************************************************/

void main(int argc, char **argv)
{
	// Initialize the OpenGL window.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(viewport_width, viewport_height);
	glutCreateWindow("Animation");

	// Set up the scene.
	init();

	// Disable key repeat (keyPressed or specialKeyPressed will only be called once when a key is first pressed).
	glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF);

	// Register GLUT callbacks.
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyPressed);
	glutSpecialFunc(specialKeyPressed);
	glutKeyboardUpFunc(keyReleased);
	glutSpecialUpFunc(specialKeyReleased);
	glutPassiveMotionFunc(mouseMoved);
	glutMotionFunc(mouseMoved);
	glutIdleFunc(idle);

	// Record when we started rendering the very first frame (which should happen after we call glutMainLoop).
	frameStartTime = (unsigned int)glutGet(GLUT_ELAPSED_TIME);

	// Enter the main drawing loop (this will never return).
	glutMainLoop();
}

/******************************************************************************
 * GLUT Callbacks (don't add any other functions here)
 ******************************************************************************/

 /*
	 Called when GLUT wants us to (re)draw the current animation frame.

	 Note: This function must not do anything to update the state of our simulated
	 world. Animation (moving or rotating things, responding to keyboard input,
	 etc.) should only be performed within the think() function provided below.
 */
void display(void)
{
	/*
		TEMPLATE: REPLACE THIS COMMENT WITH YOUR DRAWING CODE
		
		Separate reusable pieces of drawing code into functions, which you can add
		to the "Animation-Specific Functions" section below.
		
		Remember to add prototypes for any new functions to the "Animation-Specific
		Function Prototypes" section near the top of this template.
	*/

	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, renderFillEnabled ? GL_FILL : GL_LINE);

	// Update camera
	GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);
	float camX = helicopter_position.x + sin((helicopterHeading) * 3.1415f / 180.0f) * 10.0f;
	float camZ = helicopter_position.z + cos((helicopterHeading) * 3.1415f / 180.0f) * 10.0f;

	glLoadIdentity();
	
	if(orbitCamera)
	{
		cameraYaw = fmod(cameraYaw + dx * 50 * FRAME_TIME_SEC, 360.0f);
		cameraPitch += dy * 50 * FRAME_TIME_SEC;

		if (cameraPitch > 90.0f)
		{
			cameraPitch = 90.0f;
		}
		else if (cameraPitch < -90.0f)
		{
			cameraPitch = -90.0f;
		}

		gluLookAt(
			helicopter_position.x + sin((cameraYaw + helicopterHeading) * 3.1415f / 180.0f) * 10.0f,
			helicopter_position.y + sin(cameraPitch * 3.1415f / 180.0f) * 10.0f,
			helicopter_position.z + cos((cameraYaw + helicopterHeading) * 3.1415f / 180.0f) * 10.0f,
			helicopter_position.x,
			helicopter_position.y,
			helicopter_position.z,
			0,
			1,
			0);

		dx = 0;
		dy = 0;

		DRAW_STRING(0, 100, "Camera Yaw: %f Camera Pitch: %f", cameraYaw, cameraPitch);
	}
	else
	{
		gluLookAt(camX, helicopter_position.y + 5.0f, camZ, helicopter_position.x, helicopter_position.y, helicopter_position.z, 0, 1, 0);
	}

	drawScene();
}

/*
	Called when the OpenGL window has been resized.
*/
void reshape(int width, int height)
{
	viewport_width = width;
	viewport_height = height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(89, (float)width / (float)height, 0.1, 1000.0);
	glPushMatrix();

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, width, height);
}

/*
	Called each time a character key (e.g. a letter, number, or symbol) is pressed.
*/
void keyPressed(unsigned char key, int x, int y)
{
	switch (tolower(key)) {

	/*
		Keyboard-Controlled Motion Handler - DON'T CHANGE THIS SECTION

		Whenever one of our movement keys is pressed, we do two things:
		(1) Update motionKeyStates to record that the key is held down. We use
			this later in the keyReleased callback.
		(2) Update the relevant axis in keyboardMotion to set the new direction
			we should be moving in. The most recent key always "wins" (e.g. if
			you're holding down KEY_MOVE_LEFT then also pressed KEY_MOVE_RIGHT,
			our object will immediately start moving right).
	*/
	case KEY_MOVE_FORWARD:
		motionKeyStates.MoveForward = KEYSTATE_DOWN;
		keyboardMotion.Surge = MOTION_FORWARD;
		break;
	case KEY_MOVE_BACKWARD:
		motionKeyStates.MoveBackward = KEYSTATE_DOWN;
		keyboardMotion.Surge = MOTION_BACKWARD;
		break;
	case KEY_MOVE_LEFT:
		motionKeyStates.MoveLeft = KEYSTATE_DOWN;
		keyboardMotion.Sway = MOTION_LEFT;
		break;
	case KEY_MOVE_RIGHT:
		motionKeyStates.MoveRight = KEYSTATE_DOWN;
		keyboardMotion.Sway = MOTION_RIGHT;
		break;

	/*
		Other Keyboard Functions (add any new character key controls here)

		Rather than using literals (e.g. "t" for spotlight), create a new KEY_
		definition in the "Keyboard Input Handling Setup" section of this file.
		For example, refer to the existing keys used here (KEY_MOVE_FORWARD,
		KEY_MOVE_LEFT, KEY_EXIT, etc).
	*/
	case KEY_RENDER_FILL:
		renderFillEnabled = !renderFillEnabled;
		break;
	case KEY_CAMERA:
		orbitCamera = !orbitCamera;
		glutSetCursor(orbitCamera ? GLUT_CURSOR_NONE : GLUT_CURSOR_INHERIT);
		break;
	case KEY_EXIT:
		exit(0);
		break;
	}
}

/*
	Called each time a "special" key (e.g. an arrow key) is pressed.
*/
void specialKeyPressed(int key, int x, int y)
{
	switch (key) {

	/*
		Keyboard-Controlled Motion Handler - DON'T CHANGE THIS SECTION

		This works as per the motion keys in keyPressed.
	*/
	case SP_KEY_MOVE_UP:
		motionKeyStates.MoveUp = KEYSTATE_DOWN;
		keyboardMotion.Heave = MOTION_UP;
		break;
	case SP_KEY_MOVE_DOWN:
		motionKeyStates.MoveDown = KEYSTATE_DOWN;
		keyboardMotion.Heave = MOTION_DOWN;
		break;
	case SP_KEY_TURN_LEFT:
		motionKeyStates.TurnLeft = KEYSTATE_DOWN;
		keyboardMotion.Yaw = MOTION_ANTICLOCKWISE;
		break;
	case SP_KEY_TURN_RIGHT:
		motionKeyStates.TurnRight = KEYSTATE_DOWN;
		keyboardMotion.Yaw = MOTION_CLOCKWISE;
		break;

	/*
		Other Keyboard Functions (add any new special key controls here)

		Rather than directly using the GLUT constants (e.g. GLUT_KEY_F1), create
		a new SP_KEY_ definition in the "Keyboard Input Handling Setup" section of
		this file. For example, refer to the existing keys used here (SP_KEY_MOVE_UP,
		SP_KEY_TURN_LEFT, etc).
	*/
	}
}

/*
	Called each time a character key (e.g. a letter, number, or symbol) is released.
*/
void keyReleased(unsigned char key, int x, int y)
{
	switch (tolower(key)) {

	/*
		Keyboard-Controlled Motion Handler - DON'T CHANGE THIS SECTION

		Whenever one of our movement keys is released, we do two things:
		(1) Update motionKeyStates to record that the key is no longer held down;
			we need to know when we get to step (2) below.
		(2) Update the relevant axis in keyboardMotion to set the new direction
			we should be moving in. This gets a little complicated to ensure
			the controls work smoothly. When the user releases a key that moves
			in one direction (e.g. KEY_MOVE_RIGHT), we check if its "opposite"
			key (e.g. KEY_MOVE_LEFT) is pressed down. If it is, we begin moving
			in that direction instead. Otherwise, we just stop moving.
	*/
	case KEY_MOVE_FORWARD:
		motionKeyStates.MoveForward = KEYSTATE_UP;
		keyboardMotion.Surge = (motionKeyStates.MoveBackward == KEYSTATE_DOWN) ? MOTION_BACKWARD : MOTION_NONE;
		break;
	case KEY_MOVE_BACKWARD:
		motionKeyStates.MoveBackward = KEYSTATE_UP;
		keyboardMotion.Surge = (motionKeyStates.MoveForward == KEYSTATE_DOWN) ? MOTION_FORWARD : MOTION_NONE;
		break;
	case KEY_MOVE_LEFT:
		motionKeyStates.MoveLeft = KEYSTATE_UP;
		keyboardMotion.Sway = (motionKeyStates.MoveRight == KEYSTATE_DOWN) ? MOTION_RIGHT : MOTION_NONE;
		break;
	case KEY_MOVE_RIGHT:
		motionKeyStates.MoveRight = KEYSTATE_UP;
		keyboardMotion.Sway = (motionKeyStates.MoveLeft == KEYSTATE_DOWN) ? MOTION_LEFT : MOTION_NONE;
		break;

	/*
		Other Keyboard Functions (add any new character key controls here)

		Note: If you only care when your key is first pressed down, you don't have to
		add anything here. You only need to put something in keyReleased if you care
		what happens when the user lets go, like we do with our movement keys above.
		For example: if you wanted a spotlight to come on while you held down "t", you
		would need to set a flag to turn the spotlight on in keyPressed, and update the
		flag to turn it off in keyReleased.
	*/
	}
}

/*
	Called each time a "special" key (e.g. an arrow key) is released.
*/
void specialKeyReleased(int key, int x, int y)
{
	switch (key) {
	/*
		Keyboard-Controlled Motion Handler - DON'T CHANGE THIS SECTION

		This works as per the motion keys in keyReleased.
	*/
	case SP_KEY_MOVE_UP:
		motionKeyStates.MoveUp = KEYSTATE_UP;
		keyboardMotion.Heave = (motionKeyStates.MoveDown == KEYSTATE_DOWN) ? MOTION_DOWN : MOTION_NONE;
		break;
	case SP_KEY_MOVE_DOWN:
		motionKeyStates.MoveDown = KEYSTATE_UP;
		keyboardMotion.Heave = (motionKeyStates.MoveUp == KEYSTATE_DOWN) ? MOTION_UP : MOTION_NONE;
		break;
	case SP_KEY_TURN_LEFT:
		motionKeyStates.TurnLeft = KEYSTATE_UP;
		keyboardMotion.Yaw = (motionKeyStates.TurnRight == KEYSTATE_DOWN) ? MOTION_CLOCKWISE : MOTION_NONE;
		break;
	case SP_KEY_TURN_RIGHT:
		motionKeyStates.TurnRight = KEYSTATE_UP;
		keyboardMotion.Yaw = (motionKeyStates.TurnLeft == KEYSTATE_DOWN) ? MOTION_ANTICLOCKWISE : MOTION_NONE;
		break;

	/*
		Other Keyboard Functions (add any new special key controls here)

		As per keyReleased, you only need to handle the key here if you want something
		to happen when the user lets go. If you just want something to happen when the
		key is first pressed, add you code to specialKeyPressed instead.
	*/
	}
}

void mouseMoved(int x, int y)
{
	if (orbitCamera == 0)
	{
		return;
	}

	dx = (viewport_width / 2) - x;
	dy = (viewport_height / 2) - y;

	glutWarpPointer(viewport_width / 2, viewport_height / 2);
}

/*
	Called by GLUT when it's not rendering a frame.

	Note: We use this to handle animation and timing. You shouldn't need to modify
	this callback at all. Instead, place your animation logic (e.g. moving or rotating
	things) within the think() method provided with this template.
*/
void idle(void)
{
	// Wait until it's time to render the next frame.

	unsigned int frameTimeElapsed = (unsigned int)glutGet(GLUT_ELAPSED_TIME) - frameStartTime;
	if (frameTimeElapsed < FRAME_TIME)
	{
		// This frame took less time to render than the ideal FRAME_TIME: we'll suspend this thread for the remaining time,
		// so we're not taking up the CPU until we need to render another frame.
		unsigned int timeLeft = FRAME_TIME - frameTimeElapsed;
		Sleep(timeLeft);
	}

	// Begin processing the next frame.

	frameStartTime = glutGet(GLUT_ELAPSED_TIME); // Record when we started work on the new frame.

	think(); // Update our simulated world before the next call to display().

	glutPostRedisplay(); // Tell OpenGL there's a new frame ready to be drawn.
	glutSwapBuffers();
}

/******************************************************************************
 * Animation-Specific Functions (Add your own functions at the end of this section)
 ******************************************************************************/

 /*
	 Initialise OpenGL and set up our scene before we begin the render loop.
 */
void init(void)
{
	glClearColor(0.0f, 1.0f, 1.0f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float)viewport_width / (float)viewport_height, 0.1, 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_DEPTH_TEST);

	initLights();
	
	// Anything that relies on lighting or specifies normals must be initialised after initLights.
	initScene();
}

/*
	Advance our animation by FRAME_TIME milliseconds.

	Note: Our template's GLUT idle() callback calls this once before each new
	frame is drawn, EXCEPT the very first frame drawn after our application
	starts. Any setup required before the first frame is drawn should be placed
	in init().
*/
void think(void)
{
	/*
		TEMPLATE: REPLACE THIS COMMENT WITH YOUR ANIMATION/SIMULATION CODE

		In this function, we update all the variables that control the animated
		parts of our simulated world. For example: if you have a moving box, this is
		where you update its coordinates to make it move. If you have something that
		spins around, here's where you update its angle.

		NOTHING CAN BE DRAWN IN HERE: you can only update the variables that control
		how everything will be drawn later in display().

		How much do we move or rotate things? Because we use a fixed frame rate, we
		assume there's always FRAME_TIME milliseconds between drawing each frame. So,
		every time think() is called, we need to work out how far things should have
		moved, rotated, or otherwise changed in that period of time.

		Movement example:
		* Let's assume a distance of 1.0 GL units is 1 metre.
		* Let's assume we want something to move 2 metres per second on the x axis
		* Each frame, we'd need to update its position like this:
			x += 2 * (FRAME_TIME / 1000.0f)
		* Note that we have to convert FRAME_TIME to seconds. We can skip this by
		  using a constant defined earlier in this template:
			x += 2 * FRAME_TIME_SEC;

		Rotation example:
		* Let's assume we want something to do one complete 360-degree rotation every
		  second (i.e. 60 Revolutions Per Minute, or RPM).
		* Each frame, we'd need to update our object's angle like this (we'll use the
		  FRAME_TIME_SEC constant as per the example above):
			a += 360 * FRAME_TIME_SEC;

		This works for any type of "per second" change: just multiply the amount you'd
		want to move in a full second by FRAME_TIME_SEC, and add or subtract that
		from whatever variable you're updating.

		You can use this same approach to animate other things like color, opacity,
		brightness of lights, etc.
	*/

	// Update helicopter altitude
	float altitude = sqrtf(helicopterRotorSpeed) * (log(helicopterRotorSpeed / 100 + 1) / log(10)) - 10.0f;

	if (altitude < 0.0f)
	{
		altitude = 0.0f;
	}

	{
		if (altitude >= 0.0f)
		{
			GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(0, -helicopter_position.y, 0);
			glTranslatef(0, altitude, 0);
			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);
			glPopMatrix();
		}

		helicopterRotorSpeed -= 5.0f * FRAME_TIME_SEC;

		if (altitude <= 0.0f)
		{
			helicopterRotorSpeed -= 40.0f * FRAME_TIME_SEC;
		}

		if (helicopterRotorSpeed < 0.0f)
		{
			helicopterRotorSpeed = 0.0f;
		}
	}

	// Update rotors
	glPushMatrix();
	glLoadMatrixf(_helicopter_rotor_shaft->transform.m);
	glRotatef(360.0f * (helicopterRotorSpeed / 60.0f) * FRAME_TIME_SEC, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter_rotor_shaft->transform.m);
	glPopMatrix();

	glPushMatrix();
	glLoadMatrixf(_helicopter_tail_rotor_shaft->transform.m);
	glRotatef(4.0f * 360.0f * (helicopterRotorSpeed / 60.0f) * FRAME_TIME_SEC, 0, 1, 0);
	glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter_tail_rotor_shaft->transform.m);
	glPopMatrix();

	/*
		Keyboard motion handler: complete this section to make your "player-controlled"
		object respond to keyboard input.
	*/
	if (keyboardMotion.Yaw != MOTION_NONE) {
		/* TEMPLATE: Turn your object right (clockwise) if .Yaw < 0, or left (anticlockwise) if .Yaw > 0 */

		GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

		if (helicopter_position.y > 0.0f)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);

			GLfvector_t vec = get_translation(_helicopter->transform.m);

			glRotatef(FRAME_TIME_SEC * 100 * keyboardMotion.Yaw, 0, 1, 0);
			helicopterHeading = fmod(helicopterHeading + FRAME_TIME_SEC * 100 * keyboardMotion.Yaw, 360.0f);

			if (helicopterHeading < 0.0f)
			{
				helicopterHeading += 360.0f;
			}

			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);

			glPopMatrix();
		}
	}
	if (keyboardMotion.Surge != MOTION_NONE) {
		/* TEMPLATE: Move your object backward if .Surge < 0, or forward if .Surge > 0 */

		GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

		if (helicopter_position.y > 0.0f)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(0, 0, -keyboardMotion.Surge * FRAME_TIME_SEC * 10);
			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Sway != MOTION_NONE) {
		/* TEMPLATE: Move (strafe) your object left if .Sway < 0, or right if .Sway > 0 */
		GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

		if (helicopter_position.y > 0.0f)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(keyboardMotion.Sway * FRAME_TIME_SEC * 10, 0, 0);
			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Heave != MOTION_NONE) {
		/* TEMPLATE: Move your object down if .Heave < 0, or up if .Heave > 0 */
		helicopterRotorSpeed += keyboardMotion.Heave * 80.0f * FRAME_TIME_SEC;
	}
}

/*
	Initialise OpenGL lighting before we begin the render loop.
	
	Note (advanced): If you're using dynamic lighting (e.g. lights that move around, turn on or
	off, or change colour) you may want to replace this with a drawLights function that gets called
	at the beginning of display() instead of init().
*/
void initLights(void)
{
	// Simple lighting setup
	GLfloat globalAmbient[] = { 0.4f, 0.4f, 0.4f, 1 };
	GLfloat lightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };
	GLfloat ambientLight[] = { 0, 0, 0, 1 };
	GLfloat diffuseLight[] = { 1, 1, 1, 1 };
	GLfloat specularLight[] = { 1, 1, 1, 1 };
	
	// Configure global ambient lighting.
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
	
	// Configure Light 0.
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	
	// Enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	// Make GL normalize the normal vectors we supply.
	glEnable(GL_NORMALIZE);
	
	// Enable use of simple GL colours as materials.
	glEnable(GL_COLOR_MATERIAL);
}

/******************************************************************************/

void initScene(void)
{
	scenegraph_node_t* grid = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, -1.0, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(1, 1, 1)
			),
			drawGrid
		)
	);

	scenegraph_node_t* helicopter = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector4(0, 1, 0, 0),
				create_glfvector3(1, 1, 1)
			),
			NULL
		)
	);

	scenegraph_node_t* helicopter_body = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(1, 1, 2)),
			drawHelicopterBody));

	scenegraph_node_t* helicopter_tail = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0.0f, 0.5f),
				create_glfvector3(0, 0, 0),
				create_glfvector3(0.125f, 0.25f, 1.5f)),
			drawHelicopterTail));

	scenegraph_node_t* helicopter_rotor = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 1, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(2, 1, 2)
			),
			drawHelicopterRotor
		));

	scenegraph_node_t* helicopter_tail_rotor = create_node(
		create_object(
			get_transform(
				create_glfvector3(0.0f, 0.5f, 5.0f),
				create_glfvector4(0, 0, 1, 90),
				create_glfvector3(2, 2, 0.185f)
			),
			drawHelicopterRotor
		)
	);

	scene_graph = create_node(create_object(get_transform(create_glfvector3(0, 0, 0), create_glfvector3(0, 0, 0), create_glfvector3(1, 1, 1)), NULL));
	append_child(scene_graph, grid);
	append_child(scene_graph, helicopter);
	append_child(helicopter, helicopter_body);
	append_child(helicopter_body, helicopter_tail);
	append_child(helicopter_tail, helicopter_tail_rotor);
	append_child(helicopter, helicopter_rotor);

	_helicopter = helicopter->obj;
	_helicopter_rotor_shaft = helicopter_rotor->obj;
	_helicopter_tail_rotor_shaft = helicopter_tail_rotor->obj;
}

transform_t get_transform(GLfvector_t translation, GLfvector_t rotation, GLfvector_t scale)
{
	transform_t transform = { 0 };

	glPushMatrix();
	glLoadIdentity();

	glScalef(scale.x, scale.y, scale.z);
	glRotatef(rotation.w, rotation.x, rotation.y, rotation.z);
	glTranslatef(translation.x, translation.y, translation.z);

	glGetFloatv(GL_MODELVIEW_MATRIX, transform.m);

	glPopMatrix();

	return transform;
}

object_t* create_object(transform_t transform, void (*draw)())
{
	object_t* object = malloc(sizeof(object_t));

	if (object == NULL)
	{
		return NULL;
	}

	object->transform = transform;
	object->draw = draw;

	return object;
}

GLfvector_t create_glfvector4(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
	GLfvector_t glfvector;

	glfvector.x = x;
	glfvector.y = y;
	glfvector.z = z;
	glfvector.w = w;

	return glfvector;
}

GLfvector_t create_glfvector3(GLfloat x, GLfloat y, GLfloat z)
{
	return create_glfvector4(x, y, z, 0);
}

scenegraph_node_t* create_node(object_t* obj)
{
	scenegraph_node_t* node = malloc(sizeof(scenegraph_node_t));

	if (node != NULL)
	{
		node->obj = obj;
		node->children = NULL;
		node->children_count = 0;
	}

	return node;
}

void append_child(scenegraph_node_t* parent, scenegraph_node_t* child)
{
	if (child == NULL)
	{
		return;
	}

	scenegraph_node_t** children = parent->children;
	scenegraph_node_t** newChildren = malloc(sizeof(scenegraph_node_t*) * (parent->children_count + 1));

	if (newChildren == NULL)
	{
		return;
	}

	if (parent->children != NULL)
	{
		memcpy(newChildren, children, sizeof(scenegraph_node_t*) * parent->children_count);

		free(parent->children);
		parent->children = NULL;
	}

	parent->children = newChildren;

	memcpy(&newChildren[parent->children_count], &child, sizeof(scenegraph_node_t*));

	parent->children_count += 1;
}

GLfvector_t get_translation(GLfloat m[16])
{
	return create_glfvector3(m[12], m[13], m[14]);
}

void drawScene(void)
{
	// Draw scene graph
	drawNode(scene_graph);

	DRAW_STRING(0, 0, "Helicopter Position");

	GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);
	DRAW_STRING(0, 15, "x: %f y: %f z: %f", helicopter_position.x, helicopter_position.y, helicopter_position.z);

	DRAW_STRING(0, 30, "Helicopter Heading");
	DRAW_STRING(0, 45, "%.2f degrees", helicopterHeading);

	DRAW_STRING(0, 60, "Helicopter Rotor Speed:");
	DRAW_STRING(0, 75, "%f RPM", helicopterRotorSpeed);
}

void drawNode(scenegraph_node_t* node)
{
	object_t* obj = node->obj;
	scenegraph_node_t** children = node->children;
	size_t children_count = node->children_count;

	glPushMatrix();
	glMultMatrixf(obj->transform.m);

	if (obj->draw != NULL)
	{
		obj->draw();
	}

	for (size_t index = 0; index < children_count; index++)
	{
		scenegraph_node_t* child = children[index];
		drawNode(child);
	}

	glPopMatrix();
}

void drawGrid(void)
{
	for (int x = -GRID_WIDTH / 2; x < GRID_WIDTH / 2; x++)
	{
		glPushMatrix();
		glTranslatef(x, 0, 0);

		for (int z = -GRID_HEIGHT / 2; z < GRID_HEIGHT / 2; z++)
		{
			glPushMatrix();
			glTranslatef(0, 0, z);

			glBegin(GL_TRIANGLE_FAN);

			glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(1.0f, 0.0f, 0.0f);
			glVertex3f(1.0f, 0.0f, 1.0f);
			glVertex3f(1.0f, 0.0f, 1.0f);
			glVertex3f(0.0f, 0.0f, 1.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);

			glEnd();

			glPopMatrix();
		}

		glPopMatrix();
	}
}

void drawHelicopterBody(void)
{
	glColor4f(1.0, 0.0, 0.0, 1.0);
	glutSolidSphere(1.0, 20, 20);
}

void drawHelicopterTail(void)
{
	glColor4f(1.0, 1.0, 0.0, 1.0);
	glutSolidCylinder(1.0, 1.0, 20, 20);
}

void drawHelicopterRotor(void)
{
	glPushMatrix();
	glTranslatef(0, 0.1, 0);
	glRotatef(90, 1, 0, 0);
	glutSolidCylinder(0.1, 0.1, 10, 10);
	glPopMatrix();

	float vertices[][3] =
	{
		{ -1.0, 0.105, -0.05 },
		{ 1.0, 0.105, -0.05 },
		{ 1.0, 0.105, 0.05 },

		{ 1.0, 0.105, 0.05 },
		{ -1.0, 0.105, 0.05 },
		{ -1.0, 0.105, -0.05 },

		{ -0.05, 0.105, -1.0 },
		{ 0.05, 0.105, -1.0 },
		{ 0.05, 0.105, 1.0 },

		{ 0.05, 0.105, 1.0 },
		{ -0.05, 0.105, 1.0 },
		{ -0.05, 0.105, -1.0 },
	};

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &vertices);
	glDrawArrays(GL_TRIANGLES, 0, sizeof(vertices) / sizeof(vertices[0]));
	glDisableClientState(GL_VERTEX_ARRAY);
}

void drawString(float x, float y, const unsigned char* string)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, viewport_width, viewport_height, 0);

	glDisable(GL_LIGHTING);

	glColor4f(1.0, 1.0, 1.0, 1.0);
	glRasterPos2f(x + 3, y + 15); // Draw text in screen space.
	glutBitmapString(GLUT_BITMAP_8_BY_13, string);

	glEnable(GL_LIGHTING);

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
}