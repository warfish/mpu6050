
#include <stdlib.h>
#include <stdio.h>
#include <Windows.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h

#include "data.h"

// Rotation 
static const GLfloat g_angle = 1.0f;
static GLfloat g_xrot = 0.0f;
static GLfloat g_yrot = 0.0f;
static GLfloat g_zrot = 0.0f;

// Sensor data channel handle
static HANDLE gSensorHandle = INVALID_HANDLE_VALUE;

////////////////////////////////////////////////////////////////////////////////////////

#define DIE(_msg_) {							\
	DWORD _error = GetLastError();				\
	fprintf(stderr, _msg_ ": %d\n", _error);	\
	exit(_error);								\
}												\

static void usage(void)
{
	printf("mputest [COM]\n");
	printf("Arguments:\n");
	printf(" COM - Serial port MPU is connected to, defaults to COM5\n");
	printf("\n");
}

////////////////////////////////////////////////////////////////////////////////////////

void initGL(void) 
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
}

void reshape(GLsizei width, GLsizei height)
{
	if (height == 0) height = 1;
	GLfloat aspect = (GLfloat)width / (GLfloat)height;

	// Set the viewport to cover the new window
	glViewport(0, 0, width, height);

	// Set the aspect ratio of the clipping volume to match the viewport
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Enable perspective projection with fovy, aspect, zNear and zFar
	gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}

void display(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	
	glLoadIdentity();                
	glTranslatef(0.0f, 0.0f, -5.0f); 
	glRotatef(g_xrot, 1.0f, 0.0f, 0.0f);
	glRotatef(g_yrot, 0.0f, 1.0f, 0.0f);
	glRotatef(g_zrot, 0.0f, 0.0f, 1.0f);

	glBegin(GL_QUADS); 
		// Top face
		glVertex3f(1.0f, 1.0f, -1.0f);
		glVertex3f(-1.0f, 1.0f, -1.0f);
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f(1.0f, 1.0f, 1.0f);

		// Bottom face
		glVertex3f(1.0f, -1.0f, 1.0f);
		glVertex3f(-1.0f, -1.0f, 1.0f);
		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(1.0f, -1.0f, -1.0f);

		// Front face
		glVertex3f(1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, -1.0f, 1.0f);
		glVertex3f(1.0f, -1.0f, 1.0f);

		// Back face (z = -1.0f)
		glVertex3f(1.0f, -1.0f, -1.0f);
		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(-1.0f, 1.0f, -1.0f);
		glVertex3f(1.0f, 1.0f, -1.0f);

		// Left face
		glColor3f(0.0f, 0.0f, 1.0f);     // Blue
		glVertex3f(-1.0f, 1.0f, 1.0f);
		glVertex3f(-1.0f, 1.0f, -1.0f);
		glVertex3f(-1.0f, -1.0f, -1.0f);
		glVertex3f(-1.0f, -1.0f, 1.0f);

		// Right face
		glVertex3f(1.0f, 1.0f, -1.0f);
		glVertex3f(1.0f, 1.0f, 1.0f);
		glVertex3f(1.0f, -1.0f, 1.0f);
		glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();
	
	glutSwapBuffers(); 
}

void keyboardHandler(unsigned char key, int x, int y)
{
	// Wasd movement, for testing
	key = toupper(key);
	switch (toupper(key))
	{
	case 'W':
		g_xrot -= 1.0f;
		break;

	case 'S':
		g_xrot += 1.0f;
		break;
		
	case 'A':
		g_yrot -= 1.0f;
		break;

	case 'D':
		g_yrot += 1.0f;
		break;

	default:
		break;
	};

	glutPostRedisplay();
}

void keyboardSpecialHandler(int key, int x, int y)
{
	// Reset rotation on F5 key press
	if (key == GLUT_KEY_F5)
	{
		g_xrot = 0.0;
		g_yrot = 0.0;
		g_zrot = 0.0;
		glutPostRedisplay();
	}
}

void timerFunc(int value)
{
	struct MpuReport report;

	DWORD bytesRead = 0;
	BOOL res = ReadFile(gSensorHandle, &report, sizeof(report), &bytesRead, NULL);
	if (!res)
	{
		DIE("Could not read MPU report");
	}

	if (report.signature != MPU_REPORT_SIGNATURE)
	{
		fprintf(stderr, "Invalid report signature");
		goto _bad_report;
	}

	if (report.size != sizeof(report))
	{
		fprintf(stderr, "Invalid report size");
		goto _bad_report;
	}

	struct MpuReport copy = report;
	copy.crc = 0;
	if (report.crc != crc32(&copy, sizeof(copy)))
	{
		fprintf(stderr, "CRC error\n");
		goto _bad_report;
	}

	printf("Gyro {%.4f, %.4f, %.4f}\n", (double)report.x_angle, (double)report.y_angle, (double)report.z_angle);

	// Unmatched rotation axis assignment is not an error. This setup is convinient for testinh
	g_xrot = report.y_angle;
	g_yrot = report.z_angle;
	g_zrot = report.x_angle;

	glutPostRedisplay();

_bad_report:
	glutTimerFunc(100, timerFunc, 0);
}

int main(int argc, char** argv)
{
	const char* portName = NULL;
	if (argc == 1)
	{
		portName = "COM5";
	}
	else if (argc == 2)
	{
		portName = argv[1];
	}
	else
	{
		usage();
		return EXIT_FAILURE;
	}

	printf("Using serial port %s\n", portName);

	gSensorHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (gSensorHandle == INVALID_HANDLE_VALUE)
	{
		DIE("Could not open serial port");
	}

	DCB portConfig;
	if (GetCommState(gSensorHandle, &portConfig) == FALSE)
	{
		DIE("Could not get serial port state");
	}

	portConfig.BaudRate = CBR_115200;
	portConfig.StopBits = ONESTOPBIT;
	portConfig.Parity = NOPARITY;
	portConfig.ByteSize = 8;

	if (SetCommState(gSensorHandle, &portConfig) == FALSE)
	{
		DIE("Could not set serial port state");
	}

	glutInit(&argc, argv);           
	glutInitDisplayMode(GLUT_DOUBLE);
	glutInitWindowSize(640, 480);   
	glutInitWindowPosition(50, 50); 
	glutCreateWindow("MPU6050");    
	glutDisplayFunc(display);       
	glutReshapeFunc(reshape);       
	glutKeyboardFunc(keyboardHandler);
	glutSpecialFunc(keyboardSpecialHandler);
	glutTimerFunc(100, timerFunc, 0);
	initGL();                       
	glutMainLoop();                 

	CloseHandle(gSensorHandle);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////

#if 0

Calibrations:

xa = 1737.16
ya = 877.92
za = -15428.24
xg = -197.00
yg = 56.79
zg = -199.60

xa = 1591.72
ya = 782.68
za = -15473.28
xg = -190.38
yg = 51.95
zg = -195.29

xa = 1546.52
ya = 703.20
za = -15527.60
xg = -191.13
yg = 48.00
zg = -196.69

xg = -192.83
yg = 52.24
zg = -197.19

#endif // 0

