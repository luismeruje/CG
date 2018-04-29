#include<stdio.h>


#ifdef __APPLE__
#include <GLUT/glut.h>
#include "/usr/local/Cellar/devil/1.8.0_1/include/IL/il.h"
#else
#include <GL/glew.h>
#include <GL/glut.h>
#include <IL/il.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include "tinyXml/tinyxml.h"
#include "tinyXml/tinystr.h"
#include "tinyXml/tinyxmlparser.cpp"
#include "tinyXml/tinystr.cpp"
#include "tinyXml/tinyxml.cpp"
#include "tinyXml/tinyxmlerror.cpp"
#include <string>
#include <vector>
#include <iostream>
#include <limits>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;
float *vertexb;

GLuint buffers[1];

//WARNING: Princípios de encapsulamento não foram aplicados por ser um programa simples. São passados objetos entre funções e não cópias.
class Point
{
	double x, y, z;
public:

	Point(double x, double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double getX(){ return x; }
	double getY(){ return y; }
	double getZ(){ return z; }
};

class Rotate
{
    vector<Point> modelPoints;
    int time;
    Point * pivot;

public:
    Rotate(vector<Point> modelPoints,int time,Point * pivot){
        this->modelPoints = modelPoints;
        this->time = time;
        this->pivot = pivot;
    }

    vector<Point> getModelPoints(){return modelPoints;}
	int getTime(){return time;}
	Point * getPivot(){return pivot;}


	//setModelPoints
	void setModelPoints(vector<Point> modelPoints){
		this->modelPoints = modelPoints;
	}
};

vector<Point> points;
vector<Rotate> rotates;
//vector<Translate> translates;
float rotateHorizontal = 0;
float rotateVertical = 0;
float color1 = 0, color2 = 0, color3 = 0;
int pointc=0;
int mode = GL_LINE;

void changeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window with zero width).
	if(h == 0)
		h = 1;

	// compute window'angleSin aspect ratio 
	float ratio = w * 1.0 / h;

	// Set the projection matrix as current
	glMatrixMode(GL_PROJECTION);
	// Load Identity Matrix
	glLoadIdentity();
	
	// Set the viewport to be the entire window
    	glViewport(0, 0, w, h);

	// Set perspective
	gluPerspective(45.0f ,ratio, 1.0f ,1000.0f);

	// return to the model view matrix mode
	glMatrixMode(GL_MODELVIEW);
}

void loadPointsToMemory( string fileName, Matrix4d matrix ) {
	ifstream input(fileName.c_str());
	string line;
	double x, y, z;
	int num;

	if (!input)
		printf("Unable to open: %s\n", fileName.c_str());
	else {
		while (getline(input, line)) {
			sscanf(line.c_str(), "%lf;%lf;%lf", &x, &y, &z);

			Vector4d coord;
			coord << x, y, z, 1;

			coord = matrix * coord;
			points.push_back(Point(coord(0), coord(1), coord(2)));
			pointc++;
		}
	}
}

void loadPointsToRotate(string fileName, Matrix4d matrix){
    ifstream input(fileName.c_str());
    string line;
    vector <Point> rotatePoints;
    double x, y, z;
    int num;
	printf("CheckPoint2-1\n");
    if (!input)
        printf("Unable to open: %s\n", fileName.c_str());
    else {
    	printf("CheckPoint2-2\n");
        while (getline(input, line)) {
            sscanf(line.c_str(), "%lf;%lf;%lf", &x, &y, &z);

            Vector4d coord;
            coord << x, y, z, 1;

            coord = matrix * coord;
            rotatePoints.push_back(Point(coord(0), coord(1), coord(2)));

        }
        printf("CheckPoint2-3\n");
        Rotate rotateInstance = rotates.back();
        rotateInstance.setModelPoints(rotatePoints);
    }

}
/*
// given  global t, returns the point in the curve
    void getGlobalCatmullRomPoint(float gt, float *pos, float *deriv) {

        float t = gt * POINT_COUNT; // this is the real global t alterar por size do array
        int index = floor(t);  // which segment
        t = t - index; // where within  the segment

        // indices store the points
        int indices[4];
        indices[0] = (index + POINT_COUNT-1)%POINT_COUNT;
        indices[1] = (indices[0]+1)%POINT_COUNT;
        indices[2] = (indices[1]+1)%POINT_COUNT;
        indices[3] = (indices[2]+1)%POINT_COUNT;

        getCatmullRomPoint(t, p[indices[0]], p[indices[1]], p[indices[2]], p[indices[3]], pos, deriv);
    }
}


void getCatmullRomPoint(float t, float *p0, float *p1, float *p2, float *p3, float *pos, float *deriv) {

    // catmull-rom matrix
    float m[4][4] = {	{-0.5f,  1.5f, -1.5f,  0.5f},
                         { 1.0f, -2.5f,  2.0f, -0.5f},
                         {-0.5f,  0.0f,  0.5f,  0.0f},
                         { 0.0f,  1.0f,  0.0f,  0.0f}};

    // Compute A = M * P

    float a[4][4];
    for (int i =0; i< 4; i++)
        for (int j=0; j<4; j++)
            a[i][j]= m[i][0]*p0[j] +
                     m[i][1]*p1[j] +
                     m[i][2]*p2[j] +
                     m[i][3]*p3[j];
    // Compute pos = T * A
    for (int i = 0; i < 4 ; i++){
        pos[i] = pow(t,3) * a[0][i] + pow(t,2) * a[1][i] +t * a[2][i] + a[3][i];
    }

    // compute deriv = T' * A
    for (int i = 0; i < 4 ; i++){
        deriv[i] = 3* pow(t,2) * a[0][i] + 2*t * a[1][i] + a[2][i];
    }
    // ...
}
 */
	Matrix4d translateMatrix(TiXmlElement *elem, Matrix4d matrix) {
		double x, y, z;
		Matrix4d translationMatrix(4, 4);

		elem->Attribute("X", &x);
		elem->Attribute("Y", &y);
		elem->Attribute("Z", &z);

		translationMatrix << 1, 0, 0, x,
				0, 1, 0, y,
				0, 0, 1, z,
				0, 0, 0, 1;

		return translationMatrix * matrix;
	}
/*
Matrix4d  translateMatrixwtime(TiXmlElement * elem, Matrix4d matrix, Point* pontos, int time){
    double x, y, z;
    Matrix4d translationMatrix(4,4);
    //entrar em ciclo para guardar todos os pontos
    elem->Attribute( "X",&x );
    elem->Attribute( "Y",&y );
    elem->Attribute( "Z",&z );
}
*/
	Matrix4d rotateMatrixwtime(TiXmlElement *elem, Matrix4d matrix, int time) {
		double angle, x, y, z, angleCosine, angleSin;
		Matrix4d rotationMatrix(4, 4);
		angle = glutGet(GLUT_ELAPSED_TIME) % time;
		elem->Attribute("axisX", &x);
		elem->Attribute("axisY", &y);
		elem->Attribute("axisZ", &z);

		angleCosine = cos(angle * (M_PI / 180));
		angleSin = sin(angle * (M_PI / 180));

		rotationMatrix << (x * x * (1 - angleCosine) + angleCosine), (x * y * (1 - angleCosine) - z * angleSin), (
				x * z * (1 - angleCosine) + y * angleSin), 0,
				(y * x * (1 - angleCosine) + z * angleSin), (y * y * (1 - angleCosine) + angleCosine), (
				y * z * (1 - angleCosine) - x * angleSin), 0,
				(z * x * (1 - angleCosine) - y * angleSin), (z * y * (1 - angleCosine) + x * angleSin), (
				z * z * (1 - angleCosine) + angleCosine), 0,
				0, 0, 0, 1;

		return rotationMatrix * matrix;

	}
	Matrix4d rotateMatrix(TiXmlElement *elem, Matrix4d matrix) {
		double angle, x, y, z, angleCosine, angleSin;
		Matrix4d rotationMatrix(4, 4);

		elem->Attribute("angle", &angle);
		elem->Attribute("axisX", &x);
		elem->Attribute("axisY", &y);
		elem->Attribute("axisZ", &z);

		angleCosine = cos(angle * (M_PI / 180));
		angleSin = sin(angle * (M_PI / 180));

		rotationMatrix << (x * x * (1 - angleCosine) + angleCosine), (x * y * (1 - angleCosine) - z * angleSin), (
				x * z * (1 - angleCosine) + y * angleSin), 0,
				(y * x * (1 - angleCosine) + z * angleSin), (y * y * (1 - angleCosine) + angleCosine), (
				y * z * (1 - angleCosine) - x * angleSin), 0,
				(z * x * (1 - angleCosine) - y * angleSin), (z * y * (1 - angleCosine) + x * angleSin), (
				z * z * (1 - angleCosine) + angleCosine), 0,
				0, 0, 0, 1;

		return rotationMatrix * matrix;

	}
	Matrix4d scaleMatrix(TiXmlElement *elem, Matrix4d matrix) {
		double x, y, z;
		Matrix4d scalingMatrix(4, 4);

		elem->Attribute("X", &x);
		elem->Attribute("Y", &y);
		elem->Attribute("Z", &z);

		scalingMatrix << x, 0, 0, 0,
				0, y, 0, 0,
				0, 0, z, 0,
				0, 0, 0, 1;

		return scalingMatrix * matrix;
	}

	void createNewRotate(TiXmlElement *elem) {
		double x, y, z;
		vector<Point> rotatePoints;
		int time;
        elem->Attribute("axisX", &x);
        elem->Attribute("axisY", &y);
        elem->Attribute("axisZ", &z);
		elem->Attribute("time", &time);

		Point pivot = Point(x,y,z);

		Rotate newRotate = Rotate(rotatePoints,time,&pivot);

		rotates.push_back(newRotate);
	}

	void loadGroupFromXML(TiXmlElement *group, Matrix4d matrix) {
		TiXmlElement *elem;
		double angle, x, y, z;
		int translateFlag = 0, rotateFlag = 0, scaleFlag = 0,specialFlag = 0;


		for (elem = group->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement()) {
			if (strcmp(elem->Value(), "translate") == 0 && translateFlag == 0) {
				matrix = translateMatrix(elem, matrix);
				translateFlag++;
			} else if (strcmp(elem->Value(), "rotate") == 0 && rotateFlag == 0) {
                const char *isSpecialRotate = elem->Attribute("time");
				if (isSpecialRotate) {
                    createNewRotate(elem);
                    specialFlag=1;
                }
				else
					matrix = rotateMatrix(elem, matrix);
				rotateFlag++;
			} else if (strcmp(elem->Value(), "scale") == 0 && scaleFlag == 0) {
				matrix = scaleMatrix(elem, matrix);
				scaleFlag++;
			}
		}

		// Models
		elem = group->FirstChildElement("models");
		if (elem != NULL) {
            if(specialFlag == 0){
                for (elem = elem->FirstChildElement("model"); elem != NULL; elem = elem->NextSiblingElement("model")) {
                    loadPointsToMemory(elem->Attribute("file"), matrix);
                }
            }
            else if(specialFlag == 1){
            	printf("CheckPoint1\n");
                loadPointsToRotate(elem->Attribute("file"),matrix);
                printf("CheckPoint2\n");
            }
		}

		// Other groups
		for (elem = group->FirstChildElement("group"); elem != NULL; elem = elem->NextSiblingElement("group"))
			loadGroupFromXML(elem, matrix);

	}

	void loadFiguresFromXML(const char *xmlFile) {
		TiXmlDocument doc(xmlFile);
		Matrix4d matrix = Matrix4d::Identity(4, 4);

		bool loaded = doc.LoadFile();
		if (loaded) {
			for (TiXmlElement *group = doc.FirstChildElement("scene")->FirstChildElement("group");
				 group != NULL; group = group->NextSiblingElement("group"))
				loadGroupFromXML(group, matrix);
		} else
			printf("Failed to load %s \n.", xmlFile);
	}

	void handleKeyboardSpecialEvent(int key_code, int x, int y) {
		switch (key_code) {
			case GLUT_KEY_LEFT:
				rotateHorizontal += -0.1;
				if (rotateHorizontal <= -(M_PI))
					rotateHorizontal += 0.1;
				break;
			case GLUT_KEY_RIGHT:
				rotateHorizontal += 0.1;
				if (rotateHorizontal >= M_PI)
					rotateHorizontal += -0.1;
				break;
			case GLUT_KEY_DOWN:
				rotateVertical += -0.1;
				if (rotateVertical <= -(M_PI / 2))
					rotateVertical += 0.1;
				break;
			case GLUT_KEY_UP:
				rotateVertical += 0.1;
				if (rotateVertical >= M_PI / 2)
					rotateVertical += -0.1;

			default:
				break;
		}
		glutPostRedisplay();
	}

	void handleKeyboardEvent(unsigned char key, int x, int y) {
		switch (key) {
			case 'f':
				mode = GL_FILL;
				break;
			case 'l':
				mode = GL_LINE;
				break;
			case 'p':
				mode = GL_POINT;
				break;
			case '1':
				if (color1 < 1.0) color1 += 0.1;
				break;
			case '2':
				if (color2 < 1.0) color2 += 0.1;
				break;
			case '3':
				if (color3 < 1.0) color3 += 0.1;
				break;
			case '4':
				if (color1 > 0.0) color1 -= 0.1;
				break;
			case '5':
				if (color2 > 0.0) color2 -= 0.1;
				break;
			case '6':
				if (color3 > 0.0) color3 -= 0.1;
				break;
			default:
				break;
		}
		glutPostRedisplay();
	}

	void renderScene(void) {

		// clear buffers
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// set the camera
		glLoadIdentity();
		gluLookAt(250.0 * cos(rotateVertical) * sin(rotateHorizontal), 250.0 * sin(rotateVertical),
				  250.0 * cos(rotateVertical) * cos(rotateHorizontal),
				  0.0, 0.0, 0.0,
				  0.0f, 1.0f, 0.0f);

		glPolygonMode(GL_FRONT, mode);
		glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
		glDrawArrays(GL_TRIANGLES, 0, pointc);

		// End of frame
		glutSwapBuffers();
		//free(vertexb);
	}
	void prepareVBO() {

		vector<Point>::iterator it;
		glColor3f(color1, color2, color3);
		vertexb = (float *) malloc(sizeof(float) * pointc * 3);
		int pos = 0;
		for (it = points.begin(); it != points.end(); it++) {
			vertexb[pos++] = (it->getX());
			vertexb[pos++] = (it->getY());
			vertexb[pos++] = (it->getZ());

			/**
                glVertex3f(it->getX(),it->getY(),it->getZ());
                it++;
                glVertex3f(it->getX(),it->getY(),it->getZ());
                it++;
                glVertex3f(it->getX(),it->getY(),it->getZ());
             */

		}
		glGenBuffers(1, buffers);
		glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);;
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * pointc * 3, vertexb, GL_STATIC_DRAW);
		glVertexPointer(3, GL_FLOAT, 0, 0);


	}
	void initGL() {
		//  OpenGL settings
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glEnableClientState(GL_VERTEX_ARRAY);

		// Initializing component
		prepareVBO();


	}

	int main(int argc, char **argv) {

		printf("Use arrows to rotate image.\nTo change filling:\nf-> FILL\nl-> LINE\np->POINT\n");

		// init GLUT and the window
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
		glutInitWindowPosition(100, 100);
		glutInitWindowSize(800, 800);
		glutCreateWindow("CG@DI-UM");

		// Required callback registry
		glutDisplayFunc(renderScene);
		//glutIdleFunc(renderScene);
		glutReshapeFunc(changeSize);

		// Callback registration for keyboard processing
		glutKeyboardFunc(handleKeyboardEvent);
		glutSpecialFunc(handleKeyboardSpecialEvent);

#ifndef __APPLE__
		//initialize glew
		glewInit();
#endif



		//Load triangles from .3d files to be drawn
		loadFiguresFromXML(argv[1]);

		//Initialize OpenGL
		initGL();

		// enter GLUT's main cycle
		glutMainLoop();


		return 1;
	}

