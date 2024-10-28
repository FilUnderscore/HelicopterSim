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
#define KEY_FLOODLIGHT		'f'
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

void updateLightState(struct light_t* light, int index);
void drawLights(void);

void initScene(void);

void drawScene(void);
void drawNode(struct scenegraph_node_t* node);

void drawGrid(int** displayList);
void drawOcean(int** displayList);
void drawHelicopterBody(int** displayList);
void drawHelicopterTail(int** displayList);
void drawHelicopterRotor(int** displayList);
void drawHelicopterLeg(int** displayList);
void drawTreeLog(int** displayList);
void drawTreeLeaves(int** displayList);
void drawBuilding(int** displayList);
void drawWindTurbine(int** displayList);

float getGroundHeight(float x, float z);

double perlin(double x, double y, double z);
double fade(double t);
double lerp(double t, double a, double b);
double grad(int hash, double x, double y, double z);

void drawString(float x, float y, const unsigned char* string);

void loadImage(const char* filename, int* imageWidth, int* imageHeight, GLubyte** imageData);
void drawSkybox();

/******************************************************************************
 * Animation-Specific Setup (Add your own definitions, constants, and globals here)
 ******************************************************************************/

int p[512] = {
	151,160,137,91,90,15,
   131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
   190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
   88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
   77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
   102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
   135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
   5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
   223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
   129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
   251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
   49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
   138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
   151,160,137,91,90,15,
   131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
   190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
   88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
   77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
   102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
   135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
   5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
   223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
   129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
   251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
   49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
   138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};

int groundTextureWidth, groundTextureHeight;
GLubyte* groundTexture;

const char** skyboxTextureFilenames[3] =
{
	"bluecloud_bk.ppm",
	"bluecloud_rt.ppm",
	"bluecloud_lf.ppm",
};

int** skyboxTextureWidths[3];
int** skyboxTextureHeights[3];

GLubyte** skyboxTextures[3];

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
#define GRID_SQUARE_SCALE 100
#define GRID_HEIGHT_SCALE 20

#define TREE_COUNT 1500 // Lower if performance is being affected.
#define BUILDING_COUNT 100
#define WIND_TURBINE_COUNT 20

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
	void (*draw)(int**);
} object_t;

typedef struct
{
	object_t* obj;
	int* displayList;

	struct scenegraph_node_t** children;
	size_t children_count;
} scenegraph_node_t;

typedef enum
{
	directional,
	spot
} lighttype_t;

typedef struct
{
	GLfvector_t position;
	GLfvector_t color;
	GLfvector_t spotDirection;

	float intensity;
	float spotCutoffAngle;

	lighttype_t type;
	int enabled;
	int dirty;
} light_t;

scenegraph_node_t* scene_graph = NULL;
object_t* _helicopter = NULL;
object_t* _helicopter_rotor_shaft = NULL;
object_t* _helicopter_tail_rotor_shaft = NULL;

object_t* ground = NULL;
object_t** wind_turbine_rotors[WIND_TURBINE_COUNT];

light_t lights[8];

transform_t get_transform(GLfvector_t translation, GLfvector_t rotation, GLfvector_t scale);
object_t* create_object(transform_t transform, void (*draw)());
GLfvector_t create_glfvector4(GLfloat x, GLfloat y, GLfloat z, GLfloat w);
GLfvector_t create_glfvector3(GLfloat x, GLfloat y, GLfloat z);
scenegraph_node_t* create_node(object_t* obj);
void append_child(scenegraph_node_t* parent, scenegraph_node_t* child);

GLfvector_t get_translation(GLfloat m[16]);
GLfvector_t get_scale(GLfloat m[16]);

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
	drawSkybox();
	
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

		DRAW_STRING(viewport_width / 2, 190, "Camera Yaw: %f Camera Pitch: %f", cameraYaw, cameraPitch);
	}
	else
	{
		gluLookAt(camX, helicopter_position.y + 5.0f, camZ, helicopter_position.x, helicopter_position.y, helicopter_position.z, 0, 1, 0);
	}

	drawLights();
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
	gluPerspective(89, (float)width / (float)height, 0.1, 9000.0);
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
	case KEY_FLOODLIGHT:
	{
		light_t* flood_light = &lights[1];
		flood_light->enabled = !flood_light->enabled;
		flood_light->dirty = 1;
		break;
	}
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
	glClearColor(0.67f, 0.82f, 0.85f, 1.0f);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float)viewport_width / (float)viewport_height, 0.1, 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_DEPTH_TEST);

	initLights();
	
	// Anything that relies on lighting or specifies normals must be initialised after initLights.
	initScene();

	glEnable(GL_FOG);

	GLfloat fogColor[4] = { 0.67, 0.82, 0.85, 0.2 };
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogf(GL_FOG_MODE, GL_EXP);
	glFogf(GL_FOG_DENSITY, 0.0005);
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
	GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

	GLfvector_t ground_scale = get_scale(ground->transform.m);
	float ground_height = getGroundHeight(helicopter_position.x / ground_scale.x, helicopter_position.z / ground_scale.z) * ground_scale.y + 1.0f;

	float altitude = ((helicopterRotorSpeed * helicopterRotorSpeed) / 1000.0f) - 20.0f;

	if (altitude < 0.0f)
	{
		altitude = 0.0f;
	}

	{
		if (altitude >= 0.0f)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(0, -helicopter_position.y, 0);
			glTranslatef(0, ground_height + altitude, 0);
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

	// Update flood light
	light_t* flood_light = &lights[1];

	flood_light->position = helicopter_position;

	/*
		Keyboard motion handler: complete this section to make your "player-controlled"
		object respond to keyboard input.
	*/
	if (keyboardMotion.Yaw != MOTION_NONE) {
		/* TEMPLATE: Turn your object right (clockwise) if .Yaw < 0, or left (anticlockwise) if .Yaw > 0 */

		GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

		if (helicopter_position.y > ground_height)
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

		if (helicopter_position.y > ground_height)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(0, 0, -keyboardMotion.Surge * FRAME_TIME_SEC * 66);
			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Sway != MOTION_NONE) {
		/* TEMPLATE: Move (strafe) your object left if .Sway < 0, or right if .Sway > 0 */
		GLfvector_t helicopter_position = get_translation(_helicopter->transform.m);

		if (helicopter_position.y > ground_height)
		{
			glPushMatrix();
			glLoadMatrixf(_helicopter->transform.m);
			glTranslatef(keyboardMotion.Sway * FRAME_TIME_SEC * 66, 0, 0);
			glGetFloatv(GL_MODELVIEW_MATRIX, _helicopter->transform.m);
			glPopMatrix();
		}
	}
	if (keyboardMotion.Heave != MOTION_NONE) {
		/* TEMPLATE: Move your object down if .Heave < 0, or up if .Heave > 0 */
		helicopterRotorSpeed += keyboardMotion.Heave * 80.0f * FRAME_TIME_SEC;
	}

	for (int i = 0; i < sizeof(wind_turbine_rotors) / sizeof(wind_turbine_rotors[0]); i++)
	{
		object_t* wind_turbine_rotor = wind_turbine_rotors[i];

		glPushMatrix();
		glLoadMatrixf(wind_turbine_rotor->transform.m);
		glRotatef(360.0f * (60.0f / 60.0f) * FRAME_TIME_SEC, 0, 1, 0);
		glGetFloatv(GL_MODELVIEW_MATRIX, wind_turbine_rotor->transform.m);
		glPopMatrix();
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
	GLfloat globalAmbient[] = { 0.2f, 0.2f, 0.2f, 1 };
	//GLfloat lightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };
	//GLfloat ambientLight[] = { 0, 0, 0, 1 };
	//GLfloat diffuseLight[] = { 1, 1, 1, 1 };
	//GLfloat specularLight[] = { 1, 1, 1, 1 };
	
	// Configure global ambient lighting.
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmbient);
	
	// Configure Light 0.
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	//glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	
	// Enable lighting
	glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	
	// Make GL normalize the normal vectors we supply.
	glEnable(GL_NORMALIZE);
	
	// Enable use of simple GL colours as materials.
	//glEnable(GL_COLOR_MATERIAL);
}

void drawLights(void)
{
	for (int i = 0; i < (sizeof(lights) / sizeof(lights[0])); i++)
	{
		light_t* light = &lights[i];

		if (light->dirty)
		{
			updateLightState(light, i);
			light->dirty = 0;
		}

		if (!light->enabled)
		{
			continue;
		}

		GLfloat lightPosition[] = {light->position.x, light->position.y, light->position.z, (float)light->type};
		GLfloat lightDiffuseColor[] = { light->color.x * light->intensity, light->color.y * light->intensity, light->color.z * light->intensity, 1 };

		GLfloat lightAmbientColor[] = { 0, 0, 0, 1 };
		GLfloat lightSpecularColor[] = { 1, 1, 1, 1 };

		glLightfv(GL_LIGHT0 + i, GL_POSITION, lightPosition);
		glLightfv(GL_LIGHT0 + i, GL_AMBIENT, lightAmbientColor);
		glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lightDiffuseColor);
		glLightfv(GL_LIGHT0 + i, GL_SPECULAR, lightSpecularColor);

		switch (light->type)
		{
		case directional:
			break;
		case spot:
		{
			GLfloat lightSpotDirection[] = { light->spotDirection.x, light->spotDirection.y, light->spotDirection.z };
			GLfloat lightTheta = light->spotCutoffAngle;
			glLightfv(GL_LIGHT0 + i, GL_SPOT_DIRECTION, lightSpotDirection);
			glLightf(GL_LIGHT0 + i, GL_SPOT_CUTOFF, lightTheta);
			glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.01);
			break;
		}
		}
	}
}

void updateLightState(light_t* light, int index)
{
	if (light->enabled)
	{
		glEnable(GL_LIGHT0 + index);
	}
	else
	{
		glDisable(GL_LIGHT0 + index);
	}
}

/******************************************************************************/

void initScene(void)
{
	scenegraph_node_t* ocean = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(100, 1, 100)
			),
			drawOcean
		)
	);

	scenegraph_node_t* grid = create_node(
		create_object(
			get_transform(
				create_glfvector3(0, 0, 0),
				create_glfvector3(0, 0, 0),
				create_glfvector3(GRID_SQUARE_SCALE, GRID_HEIGHT_SCALE, GRID_SQUARE_SCALE)
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
				create_glfvector3(-1.0f, 0.0f, 0.95f),
				create_glfvector4(0, 0, 1, 90),
				create_glfvector3(2, 2, 0.185f)
			),
			drawHelicopterRotor
		)
	);

	scenegraph_node_t* helicopter_leg_l = create_node(
		create_object(
			get_transform(
				create_glfvector3(-0.5f, -0.5f, 0.0f),
				create_glfvector4(1, 0, 0, 90),
				create_glfvector3(1, 1, 3)
			),
			drawHelicopterLeg
		)
	);

	scenegraph_node_t* helicopter_leg_r = create_node(
		create_object(
			get_transform(
				create_glfvector3(0.5f, -0.5f, 0.0f),
				create_glfvector4(1, 0, 0, 90),
				create_glfvector3(1, 1, 3)
			),
			drawHelicopterLeg
		)
	);

	scene_graph = create_node(create_object(get_transform(create_glfvector3(0, 0, 0), create_glfvector3(0, 0, 0), create_glfvector3(1, 1, 1)), NULL));
	append_child(scene_graph, grid);
	append_child(scene_graph, ocean);
	append_child(scene_graph, helicopter);
	append_child(helicopter, helicopter_body);
	append_child(helicopter_body, helicopter_tail);
	append_child(helicopter_tail, helicopter_tail_rotor);
	append_child(helicopter, helicopter_rotor);
	append_child(helicopter_body, helicopter_leg_l);
	append_child(helicopter_body, helicopter_leg_r);

	_helicopter = helicopter->obj;
	_helicopter_rotor_shaft = helicopter_rotor->obj;
	_helicopter_tail_rotor_shaft = helicopter_tail_rotor->obj;

	ground = grid->obj;

	for (int i = 0; i < TREE_COUNT; i++)
	{
		GLfvector_t ground_scale = get_scale(ground->transform.m);

		float minX = -(GRID_WIDTH / 2) * ground_scale.x;
		float maxX = (GRID_WIDTH / 2) * ground_scale.x;

		float minZ = -(GRID_HEIGHT / 2) * ground_scale.z;
		float maxZ = (GRID_HEIGHT / 2) * ground_scale.z;

		float x = minX + ((maxX - minX) * ((float)rand() / (float)RAND_MAX));
		float z = minZ + ((maxZ - minZ) * (float)rand() / (float)RAND_MAX);
		float y = getGroundHeight(x / ground_scale.x, z / ground_scale.z) * ground_scale.y + 20.0f;

		if (y < 50.0f)
		{
			i--;
			continue;
		}

		scenegraph_node_t* tree = create_node(
			create_object(
				get_transform(
					create_glfvector3(x, y, z),
					create_glfvector3(0, 0, 0),
					create_glfvector3(1, 1, 1)
				),
				NULL
			)
		);

		scenegraph_node_t* tree_log = create_node(
			create_object(
				get_transform(
					create_glfvector3(0, 0, 0),
					create_glfvector4(1, 0, 0, 90),
					create_glfvector3(1, 2, 1)
				),
				drawTreeLog
			)
		);

		scenegraph_node_t* tree_leaves = create_node(
			create_object(
				get_transform(
					create_glfvector3(0, 30, 0),
					create_glfvector3(0, 0, 0),
					create_glfvector3(20, 40, 20)
				),
				drawTreeLeaves
			)
		);

		append_child(tree, tree_log);
		append_child(tree, tree_leaves);
		append_child(scene_graph, tree);
	}

	for (int i = 0; i < BUILDING_COUNT; i++)
	{
		GLfvector_t ground_scale = get_scale(ground->transform.m);

		float minX = -(GRID_WIDTH / 2) * ground_scale.x;
		float maxX = (GRID_WIDTH / 2) * ground_scale.x;

		float minZ = -(GRID_HEIGHT / 2) * ground_scale.z;
		float maxZ = (GRID_HEIGHT / 2) * ground_scale.z;

		float x = minX + ((maxX - minX) * ((float)rand() / (float)RAND_MAX));
		float z = minZ + ((maxZ - minZ) * (float)rand() / (float)RAND_MAX);
		float y = getGroundHeight(x / ground_scale.x, z / ground_scale.z) * ground_scale.y + 20.0f;

		if (y < 50.0f)
		{
			i--;
			continue;
		}

		float xScale = 20.0f + ((100.0f - 20.0f) * ((float)rand() / (float)RAND_MAX));
		float zScale = 20.0f + ((100.0f - 20.0f) * ((float)rand() / (float)RAND_MAX));
		float yScale = 50.0f + ((200.0f - 50.0f) * ((float)rand() / (float)RAND_MAX));

		scenegraph_node_t* building = create_node(
			create_object(
				get_transform(
					create_glfvector3(x, y, z),
					create_glfvector3(0, 0, 0),
					create_glfvector3(xScale, yScale, zScale)
				),
				drawBuilding
			)
		);

		append_child(scene_graph, building);
	}

	for (int i = 0; i < sizeof(wind_turbine_rotors) / sizeof(wind_turbine_rotors[0]); i++)
	{
		GLfvector_t ground_scale = get_scale(ground->transform.m);

		float minX = -(GRID_WIDTH / 2) * ground_scale.x;
		float maxX = (GRID_WIDTH / 2) * ground_scale.x;

		float minZ = -(GRID_HEIGHT / 2) * ground_scale.z;
		float maxZ = (GRID_HEIGHT / 2) * ground_scale.z;

		float x = minX + ((maxX - minX) * ((float)rand() / (float)RAND_MAX));
		float z = minZ + ((maxZ - minZ) * (float)rand() / (float)RAND_MAX);
		float y = getGroundHeight(x / ground_scale.x, z / ground_scale.z) * ground_scale.y + 20.0f;

		if (y < 700.0f)
		{
			i--;
			continue;
		}

		scenegraph_node_t* wind_turbine = create_node(
			create_object(
				get_transform(
					create_glfvector3(x, y, z),
					create_glfvector3(0, 0, 0),
					create_glfvector3(1, 1, 1)
				),
				NULL
			)
		);

		scenegraph_node_t* wind_turbine_pole = create_node(
			create_object(
				get_transform(
					create_glfvector3(0, 0, 0),
					create_glfvector4(0, 1, 0, 90),
					create_glfvector3(20, 200, 20)
				),
				drawWindTurbine
			)
		);

		scenegraph_node_t* wind_turbine_rotor = create_node(
			create_object(
				get_transform(
					create_glfvector3(0, 190, 0),
					create_glfvector4(0, 0, 1, 90),
					create_glfvector3(50, 50, 50)
				),
				drawHelicopterRotor
			)
		);

		append_child(wind_turbine, wind_turbine_pole);
		append_child(wind_turbine, wind_turbine_rotor);
		append_child(scene_graph, wind_turbine);

		wind_turbine_rotors[i] = wind_turbine_rotor->obj;
	}

	light_t* sun_light = &lights[0];
	sun_light->enabled = 1;
	sun_light->color = create_glfvector3(1, 1, 1);
	sun_light->intensity = 1.0;
	sun_light->type = directional;
	sun_light->position = create_glfvector3(5, 5, 0);
	sun_light->dirty = 1;

	light_t* flood_light = &lights[1];

	flood_light->enabled = 1;
	flood_light->color = create_glfvector3(1, 1, 1);
	flood_light->intensity = 100.0;
	flood_light->type = spot;
	GLfvector_t floodLightDir = create_glfvector3(0, -1, 0);
	flood_light->spotDirection = floodLightDir;
	flood_light->spotCutoffAngle = 30;
	flood_light->position = get_translation(_helicopter->transform.m);
	flood_light->dirty = 1;

	loadImage("grass.ppm", &groundTextureWidth, &groundTextureHeight, &groundTexture);

	for (int i = 0; i < 3; i++)
	{
		loadImage(skyboxTextureFilenames[i], &skyboxTextureWidths[i], &skyboxTextureHeights[i], &skyboxTextures[i]);
	}
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
		node->displayList = NULL;
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

GLfvector_t get_scale(GLfloat m[16])
{
	float x = sqrtf(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]);
	float y = sqrtf(m[1] * m[1] + m[5] * m[5] + m[9] * m[9]);
	float z = sqrtf(m[2] * m[2] + m[6] * m[6] + m[10] * m[10]);

	return create_glfvector3(x, y, z);
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

	DRAW_STRING(0, 100, "Controls:");
	DRAW_STRING(0, 115, "UP/DOWN ARROWS: Increase (move up) or decrease (move down) altitude.");
	DRAW_STRING(0, 130, "LEFT/RIGHT ARROWS: Turn left/right.");
	DRAW_STRING(0, 145, "W/S: Move forward/backward.");
	DRAW_STRING(0, 160, "A/D: Strafe left/right.");
	DRAW_STRING(0, 175, "L: Toggle wireframe mode.");
	DRAW_STRING(0, 190, "C: Toggle camera mode.");
	DRAW_STRING(0, 205, "F: Toggle helicopter flood light.");
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
		obj->draw(&node->displayList);
	}

	for (size_t index = 0; index < children_count; index++)
	{
		scenegraph_node_t* child = children[index];
		drawNode(child);
	}

	glPopMatrix();
}

float getGroundHeight(float x, float z)
{
	return (perlin(x / GRID_WIDTH, 0.5, z / GRID_WIDTH) + 0.25) * 100;
}

void drawPlane(int width, int height, int usePerlin)
{
	for (int x = -width / 2; x < width / 2; x++)
	{
		glPushMatrix();
		glTranslatef(x, 0, 0);

		for (int z = -height / 2; z < height / 2; z++)
		{
			glPushMatrix();
			glTranslatef(0, 0, z);

			glBegin(GL_TRIANGLE_FAN);

			double y0 = 0, y1 = 0, y2 = 0, y3 = 0;

			if (usePerlin)
			{
				y0 = getGroundHeight(x, z);
				y1 = getGroundHeight(x + 1, z);
				y2 = getGroundHeight(x, z + 1);
				y3 = getGroundHeight(x + 1, z + 1);
			}

			glNormal3f(0, 1, 0);
			glTexCoord2f(0, 0);
			glVertex3f(0.0f, y0, 0.0f);
			glNormal3f(0, 1, 0);
			glTexCoord2f(1, 0);
			glVertex3f(1.0f, y1, 0.0f);
			glNormal3f(0, 1, 0);
			glTexCoord2f(1, 1);
			glVertex3f(1.0f, y3, 1.0f);
			glNormal3f(0, 1, 0);
			glTexCoord2f(1, 1);
			glVertex3f(1.0f, y3, 1.0f);
			glNormal3f(0, 1, 0);
			glTexCoord2f(0, 1);
			glVertex3f(0.0f, y2, 1.0f);
			glNormal3f(0, 1, 0);
			glTexCoord2f(0, 0);
			glVertex3f(0.0f, y0, 0.0f);

			glEnd();

			glPopMatrix();
		}

		glPopMatrix();
	}
}

void drawGrid(int** displayList)
{
	GLfloat diffuseMat[] = { 0.0, 1.0, 0.0, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	GLfloat shine = 0.5;

	//glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	//glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);
	//glMaterialf(GL_FRONT, GL_SHININESS, shine);

	glEnable(GL_TEXTURE_2D);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, groundTextureWidth, groundTextureHeight, GL_RGB, GL_UNSIGNED_BYTE, groundTexture);

	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		drawPlane(GRID_WIDTH, GRID_HEIGHT, 1);

		glEndList();
	}

	glCallList(*displayList);

	glDisable(GL_TEXTURE_2D);
}

void drawOcean(int** displayList)
{
	GLfloat diffuseMat[] = { 0.0, 0.1, 1.0, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	GLfloat shine = 0.1;

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);
	glMaterialf(GL_FRONT, GL_SHININESS, shine);

	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		drawPlane(GRID_WIDTH, GRID_HEIGHT, 0);

		glEndList();
	}

	glCallList(*displayList);
}

void drawHelicopterBody(int** displayList)
{
	GLfloat diffuseMat[] = {1.0, 0.0, 0.0, 1.0};

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glutSolidSphere(1.0, 20, 20);
}

void drawHelicopterTail(int** displayList)
{
	GLfloat diffuseMat[] = { 1.0, 1.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glutSolidCylinder(1.0, 1.0, 20, 20);
}

void drawHelicopterRotor(int** displayList)
{
	GLfloat diffuseMat[] = { 1.0, 1.0, 1.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);

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

void drawHelicopterLeg(int** displayList)
{
	GLfloat diffuseMat[] = { 0.0, 0.0, 1.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);

	glutSolidCylinder(0.2, 0.5, 10, 10);
}

double perlin(double x, double y, double z)
{
	int X = (int)floor(x) & 255;
	int Y = (int)floor(y) & 255;
	int Z = (int)floor(z) & 255;

	x -= floor(x);
	y -= floor(y);
	z -= floor(z);

	double u = fade(x);
	double v = fade(y);
	double w = fade(z);

	int A = p[X] + Y;
	int AA = p[A] + Z;
	int AB = p[A + 1] + Z;
	int B = p[X + 1] + Y;
	int BA = p[B] + Z;
	int BB = p[B + 1] + Z;

	double ug0 = grad(p[AA], x, y, z);
	double ug1 = grad(p[BA], x - 1, y, z);
	double lug0g1 = lerp(u, ug0, ug1);

	double ug2 = grad(p[AB], x, y - 1, z);
	double ug3 = grad(p[BB], x - 1, y - 1, z);
	double lug2g3 = lerp(u, ug2, ug3);

	double lvug0g1g2g3 = lerp(v, lug0g1, lug2g3);

	double ug4 = grad(p[AA + 1], x, y, z - 1);
	double ug5 = grad(p[BA + 1], x - 1, y, z - 1);
	double lug4g5 = lerp(u, ug4, ug5);

	double ug6 = grad(p[AB + 1], x, y - 1, z - 1);
	double ug7 = grad(p[BB + 1], x - 1, y - 1, z - 1);
	double lug6g7 = lerp(u, ug6, ug7);

	double lvug4g5g6g7 = lerp(v, lug4g5, lug6g7);

	return lerp(w, lvug0g1g2g3, lvug4g5g6g7);
}

double fade(double t)
{
	return t * t * t * (t * (t * 6 - 15) + 10);
}

double lerp(double t, double a, double b)
{
	return a + t * (b - a);
}

double grad(int hash, double x, double y, double z)
{
	int h = hash & 15;
	double u = h < 8 ? x : y;
	double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);

	return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
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

/*
 * Loads a PPM image
 */
void loadImage(const char* filename, int* imageWidth, int* imageHeight, GLubyte** imageData)
{
	// the ID of the image file
	FILE* fileID;

	// maxValue
	int  maxValue;

	// total number of pixels in the image
	int  totalPixels;

	// temporary character
	char tempChar;

	// counter variable for the current pixel in the image
	int i;

	// array for reading in header information
	char headerLine[100];

	// if the original values are larger than 255
	float RGBScaling;

	// temporary variables for reading in the red, green and blue data of each pixel
	char red, green, blue;

	int filePos = 0;
	int bufferPos;

	// open the image file for reading - note this is hardcoded would be better to provide a parameter which
	//is the file name. There are 3 PPM files you can try out mount03, sky08 and sea02.
	errno_t err = fopen_s(&fileID, filename, "r");

	if (err != 0)
	{
		printf("Failed to load image %s.", filename);
		exit(0);
	}

	fseek(fileID, 0, SEEK_END);
	long fileSize = ftell(fileID);
	rewind(fileID);

	char* buffer = calloc(1, fileSize);
	fread(buffer, fileSize, 1, fileID);
	fclose(fileID);

	// read in the first header line
	//    - "%[^\n]"  matches a string of all characters not equal to the new line character ('\n')
	//    - so we are just reading everything up to the first line break
	sscanf_s(buffer, "%[^\n] %n", headerLine, sizeof(headerLine), &bufferPos);
	filePos += bufferPos;

	// make sure that the image begins with 'P3', which signifies a PPM file
	if ((headerLine[0] != 'P') || (headerLine[1] != '6'))
	{
		printf("This is not a PPM file!\n %c", headerLine[1]);
		exit(0);
	}

	// we have a PPM file
	printf("This is a PPM file\n");

	// read in the first character of the next line
	sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	filePos += bufferPos;

	// while we still have comment lines (which begin with #)
	while (tempChar == '#')
	{
		// read in the comment
		sscanf_s(&buffer[filePos], "%[^\n] %n", headerLine, sizeof(headerLine), &bufferPos);
		filePos += bufferPos;

		// print the comment
		printf("%s\n", headerLine);

		// read in the first character of the next line
		sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	}

	// read in the image hieght, width and the maximum value
	sscanf_s(&buffer[filePos], "%d %d %d%n", imageWidth, imageHeight, &maxValue, &bufferPos);
	filePos += bufferPos;

	sscanf_s(&buffer[filePos], "%c%n", &tempChar, sizeof(tempChar), &bufferPos);
	filePos += bufferPos;

	// print out the information about the image file
	printf("%d rows  %d columns  max value= %d\n", *imageHeight, *imageWidth, maxValue);

	// compute the total number of pixels in the image
	totalPixels = (*imageWidth) * (*imageHeight);

	// allocate enough memory for the image  (3*) because of the RGB data
	*imageData = calloc(totalPixels, 3 * sizeof(GLubyte));

	// determine the scaling for RGB values
	RGBScaling = 255.0 / maxValue;


	// if the maxValue is 255 then we do not need to scale the 
	//    image data values to be in the range or 0 to 255
	if (maxValue == 255)
	{
		for (i = 0; i < totalPixels; i++)
		{
			// read in the current pixel from the file
			red = buffer[filePos++];
			green = buffer[filePos++];
			blue = buffer[filePos++];

			// store the red, green and blue data of the current pixel in the data array
			(*imageData)[3 * totalPixels - 3 * i - 3] = red;
			(*imageData)[3 * totalPixels - 3 * i - 2] = green;
			(*imageData)[3 * totalPixels - 3 * i - 1] = blue;

			//(*imageData)[3 * i] = red;
			//(*imageData)[(3 * i) + 1] = green;
			//(*imageData)[(3 * i) + 2] = blue;
		}
	}
	else  // need to scale up the data values
	{
		for (i = 0; i < totalPixels; i++)
		{
			// read in the current pixel from the file
			fscanf_s(fileID, "%d %d %d", &red, &green, &blue);

			// store the red, green and blue data of the current pixel in the data array
			(*imageData)[3 * totalPixels - 3 * i - 3] = red * RGBScaling;
			(*imageData)[3 * totalPixels - 3 * i - 2] = green * RGBScaling;
			(*imageData)[3 * totalPixels - 3 * i - 1] = blue * RGBScaling;
		}
	}


	// close the image file
	fclose(fileID);
}

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

void drawTreeLog(int** displayList)
{
	GLfloat diffuseMat[] = { 0.38, 0.23, 0.08, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);
	
	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		glutSolidCylinder(10.0, 20.0, 10, 10);

		glEndList();
	}

	glCallList(*displayList);
}

void drawTreeLeaves(int** displayList)
{
	GLfloat diffuseMat[] = { 0.0, 1.0, 0.0, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		glutSolidSphere(1.0, 10, 10);

		glEndList();
	}

	glCallList(*displayList);
}

void drawBuilding(int** displayList)
{
	GLfloat diffuseMat[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		glutSolidCube(1.0);

		glEndList();
	}

	glCallList(*displayList);
}

void drawWindTurbine(int** displayList)
{
	GLfloat diffuseMat[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat ambientMat[] = { 0.0, 0.0, 0.0, 1.0 };

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMat);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambientMat);

	if (*displayList == NULL)
	{
		*displayList = glGenLists(1);

		glNewList(*displayList, GL_COMPILE);

		glutSolidCylinder(1.0, 1.0, 20, 20);

		glEndList();
	}

	glCallList(*displayList);
}