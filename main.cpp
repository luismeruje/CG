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
#include <regex>
#include <limits>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

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

vector<Point> points;

float rotateHorizontal = 0;
float rotateVertical = 0;
float color1 = 0, color2 = 0, color3 = 0;

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

void loadPointsToMemory( string fileName, Matrix4d matrix ){
	ifstream input( fileName.c_str() );
	string line;
	regex numberReg("-?[0-9]+\\.?[0-9]*");
	double x, y, z;
	int num;
	regex_iterator<string::iterator> end;

	if ( input != NULL ) {
		while ( getline( input,line ) ) { 
			num = 0;
			regex_iterator<string::iterator> it ( line.begin(), line.end(), numberReg);
			while (it != end) {
				switch (num) {
					case 0:
						x = stod(it->str());
						break;
					case 1:
						y = stod(it->str());
						break;
					case 2:
						z = stod(it->str());
						break;
					default:
						break;
				}
				num++;
				it++;
			}
			
			Vector4d coord;
			coord << x, y, z, 1;

			coord = matrix * coord;
			points.push_back( Point( coord(0),coord(1),coord(2) ));
		}
	}
	else
		printf("Unable to open: %s\n",fileName.c_str());
}

Matrix4d translateMatrix( TiXmlElement * elem, Matrix4d m ) {
	double x, y, z;
	Matrix4d t(4,4);

	elem->Attribute( "X",&x );
	elem->Attribute( "Y",&y );
	elem->Attribute( "Z",&z );
		
	t <<	1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;
	
	return t * m;	
}

Matrix4d rotateMatrix( TiXmlElement * elem, Matrix4d m ) {
	double angle, x, y, z, c, s;
	Matrix4d r(4,4);
	
	elem->Attribute( "angle",&angle );
	elem->Attribute( "axisX",&x );
	elem->Attribute( "axisY",&y );
	elem->Attribute( "axisZ",&z );

	c = cos( angle * (M_PI / 180) );
	s = sin( angle * (M_PI / 180) );

	r <<	( x * x * (1 - c) + c ),	( x * y * (1 - c) - z * s ),	( x * z * (1 - c) + y * s ),	0,
		( y * x * (1 - c) + z * s ),	( y * y * (1 - c) + c),		( y * z * (1 - c) - x * s ),	0,
		( z * x * (1 - c) - y * s ),	( z * y * (1 - c) + x * s ),	( z * z * (1 - c) + c ),	0,
		0,				0,				0,				1; 
	
	return r * m;	
}

Matrix4d scaleMatrix( TiXmlElement * elem, Matrix4d m ) {
	double x, y, z;
	Matrix4d s(4,4);

	elem->Attribute( "X",&x );
	elem->Attribute( "Y",&y );
	elem->Attribute( "Z",&z );

	s <<	x, 0, 0, 0,
		0, y, 0, 0,
		0, 0, z, 0,
		0, 0, 0, 1;
	
	return s * m;
}

void loadGroupFromXML( TiXmlElement * group, Matrix4d m ) {
	TiXmlElement * elem;
	double angle, x, y, z;
	int translateFlag = 0, rotateFlag = 0, scaleFlag = 0;
	
	
	for ( elem = group->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement() ) {
		if ( strcmp( elem->Value(),"translate" ) == 0 && translateFlag == 0 ) {
			m = translateMatrix( elem,m );
			translateFlag++;
		} 
		else if ( strcmp( elem->Value(),"rotate" ) == 0 && rotateFlag == 0 ) {
			m = rotateMatrix( elem,m );
			rotateFlag++;
		}
		else if ( strcmp( elem->Value(),"scale" ) == 0 && scaleFlag == 0 ) {
			m = scaleMatrix( elem,m );
			scaleFlag++;
		}
	}

	// Models
	elem = group->FirstChildElement("models");
	if( elem != NULL ) {
		for( elem = elem->FirstChildElement("model"); elem != NULL; elem = elem->NextSiblingElement("model")){
			loadPointsToMemory( elem->Attribute( "file" ),m );
		}	
	}

	// Other groups
	for( elem = group->FirstChildElement("group"); elem != NULL; elem = elem->NextSiblingElement("group") )
		loadGroupFromXML( elem,m );
}

void loadFiguresFromXML(const char * xmlFile){
	TiXmlDocument doc(xmlFile);
	Matrix4d m = Matrix4d::Identity(4,4);

	bool loaded = doc.LoadFile();
	if(loaded)
		for( TiXmlElement * group = doc.FirstChildElement("scene")->FirstChildElement("group"); group != NULL; group = group->NextSiblingElement("group") )
			loadGroupFromXML( group,m );
	else
	 	printf("Failed to load %s \n.",xmlFile);
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
	gluLookAt(250.0 * cos(rotateVertical) * sin(rotateHorizontal),250.0 * sin(rotateVertical),250.0 * cos(rotateVertical)*cos(rotateHorizontal), 
		      0.0,0.0,0.0,
			  0.0f,1.0f,0.0f);

	glPolygonMode(GL_FRONT,mode);

	vector<Point>::iterator it;

	for( it = points.begin() ; it != points.end() ; it++ ) {
		glColor3f(color1, color2, color3 );	
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
