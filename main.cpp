#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
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
#include <cstdio>

class Point
{

	double x, y, z;
public:

	Point(double x, double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double getX(){
		return x;
	}

	double getY(){
		return y;
	}

	double getZ(){
		return z;
	}
};


std::vector<std::string> ficheiros;
std::vector<Point> pontos;
float rotateHorizontal = 0;
float rotateVertical = 0;
int mode = GL_LINE;

void changeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window with zero width).
	if(h == 0)
		h = 1;

	// compute window's aspect ratio 
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

void loadPointsToMemory(std::string fileName){
	std::ifstream input(fileName.c_str());
	std::string line;
	double x, y, z;
	int num;
	if(input){
		while(std::getline(input,line)){
			std::sscanf(line.c_str(),"%lf;%lf;%lf",&x,&y,&z);
			pontos.push_back(Point(x,y,z));
		}
	}
	else
		printf("Unable to open: %s\n",fileName.c_str());
}

void loadFiguresFromXML(const char * xmlFile){
	TiXmlDocument doc(xmlFile);

	bool loaded = doc.LoadFile();
	if(loaded){
		
		for(TiXmlElement * e = doc.FirstChildElement("scene")->FirstChildElement("model"); e != NULL; e = e->NextSiblingElement("model")){
			ficheiros.push_back(e->Attribute("file"));
		}
		for (int i = 0; i < ficheiros.size();i++){
			loadPointsToMemory(ficheiros.at(i));
		}
	}
	else{
	 	printf("Failed to load %s \n.",xmlFile);
	}
	
}




void handleKeyboardSpecialEvent(int key_code,int x, int y){
	switch(key_code){
		case GLUT_KEY_LEFT:
			rotateHorizontal += -0.1;
			if(rotateHorizontal <= -(M_PI))
				rotateHorizontal += 0.1;
			break;
		case GLUT_KEY_RIGHT:
			rotateHorizontal += 0.1;
			if(rotateHorizontal >= M_PI)
				rotateHorizontal += -0.1;
			break;
		case GLUT_KEY_DOWN:
			rotateVertical += -0.1;
			if(rotateVertical <= -(M_PI / 2))
				rotateVertical += 0.1;
			break;
		case GLUT_KEY_UP:
			rotateVertical += 0.1;
			if(rotateVertical >= M_PI / 2)
				rotateVertical += -0.1;

		default:
			break;
	}
	glutPostRedisplay();
}

void handleKeyboardEvent(unsigned char key, int x, int y){
	switch(key){
		case 'f':
			mode = GL_FILL;
			break;
		case 'l':
			mode = GL_LINE;
			break;
		case 'p':
			mode = GL_POINT;
			break;
		default:
			break;
	}
	glutPostRedisplay();
}

void renderScene(void) {

	// clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// set the camera
	glLoadIdentity();
	gluLookAt(6.0 * cos(rotateVertical) * sin(rotateHorizontal),6.0 * sin(rotateVertical),6.0*cos(rotateVertical)*cos(rotateHorizontal), 
		      0.0,0.0,0.0,
			  0.0f,1.0f,0.0f);

	glPolygonMode(GL_FRONT,mode);

	//Draw triangles
	std::vector<Point>::iterator it;
	for(it = pontos.begin();it != pontos.end(); it++){
		glBegin(GL_TRIANGLES);
			glVertex3f(it->getX(),it->getY(),it->getZ());
			it++;
			glVertex3f(it->getX(),it->getY(),it->getZ());
			it++;
			glVertex3f(it->getX(),it->getY(),it->getZ());
		glEnd();
	}

	// End of frame
	glutSwapBuffers();
}

int main(int argc, char **argv) {
	printf("Use arrows to rotate image.\nTo change filling:\nf-> FILL\nl-> LINE\np->POINT\n");
// init GLUT and the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(800,800);
	glutCreateWindow("CG@DI-UM");
		
// Required callback registry 
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	
// Callback registration for keyboard processing
	glutKeyboardFunc(handleKeyboardEvent);
	glutSpecialFunc(handleKeyboardSpecialEvent);

//  OpenGL settings
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
//Load triangles from .3d files to be drawn
//Verificar índice do argv
	loadFiguresFromXML(argv[1]);

// enter GLUT's main cycle
	glutMainLoop();
	
	return 1;
}
