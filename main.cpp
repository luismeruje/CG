#include<stdio.h>
#include <stdlib.h> 

#ifdef __APPLE__
#include <GLUT/glut.h>
#include "/usr/local/Cellar/devil/1.8.0_1/include/IL/il.h"
#else
#include <GL/glew.h>
#include <GL/glut.h>
#include <IL/il.h>
#include <tr1/memory>
using namespace std::tr1;
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
float *normalb;
float *normalv;
vector<float> changingPoints;

GLuint buffers[4];

int nrOfLights = 0;
//Four doubles per light
vector<double> posOrDirOfLights;


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
    vector<float> modelPoints;
    int time;
    float * pivot;
    Matrix4d matrix;

public:
    Rotate(int time,shared_ptr<Point> pivot, Matrix4d matrix){
        this->time = time;
        this->pivot = (float *)malloc(sizeof(float) * 3);
        this->pivot[0] = pivot->getX();
        this->pivot[1] = pivot->getY();
        this->pivot[2] = pivot->getZ();
        this->matrix = matrix;
    }

    vector<float> * getModelPoints(){return &modelPoints;}
	int getTime(){return time;}
	float*  getPivot(){return pivot;}
	Matrix4d getMatrix(){return matrix;}

};

class DrawSegment
{
	float diffR, diffG, diffB;
	float ambR, ambG, ambB;
	float specR, specG, specB;
	float emR, emG, emB;
	int beginIndex;
	int nrVertex;

public:
	DrawSegment(int beginIndex, int nrVertex){
		//Almost white
		this->ambR=0.2; this->ambG=0.2; this->ambB=0.2;
		
		//Almost black
		this->diffR=0.8; this->diffG=0.8; this->diffB=0.8;

		//White
		this->specR=0; this->specG=0; this->specB=0;

		//White
		this->emR=0; this->emG=0; this->emB=0;

		this->beginIndex = beginIndex;
		this->nrVertex = nrVertex;
	}

	void setAmbR(float ambR){this->ambR = ambR;}
	void setAmbG(float ambG){this->ambG = ambG;}
	void setAmbB(float ambB){this->ambB = ambB;}
	void setDiffR(float diffR){this->diffR = diffR;}
	void setDiffG(float diffG){this->diffG = diffG;}
	void setDiffB(float diffB){this->diffB = diffB;}
	void setSpecR(float specR){this->specR = specR;}
	void setSpecG(float specG){this->specG = specG;}
	void setSpecB(float specB){this->specB = specB;}
	void setEmR(float emR){this->emR = emR;}
	void setEmG(float emG){this->emG = emG;}
	void setEmB(float emB){this->emB = emB;}
	float getAmbR() {return ambR;}
	float getAmbG() {return ambG;}
	float getAmbB() {return ambB;}
	float getDiffR(){return diffR;}
	float getDiffG(){return diffG;}
	float getDiffB(){return diffB;}
	float getSpecR(){return specR;}
	float getSpecG(){return specG;}
	float getSpecB(){return specB;}
	float getEmR(){return emR;}
	float getEmG(){return emG;}
	float getEmB(){return emB;}
	float getBeginIndex(){return beginIndex;}
	float getNrVertex(){return nrVertex;}
};

vector<Point> points;
vector< shared_ptr<Rotate> > rotates;
vector<DrawSegment> segments;
int nrOfRotates = 0;

//vector<Translate> translates;
float rotateHorizontal = 0;
float rotateVertical = 0;
float color1 = 1, color2 = 1, color3 = 1;
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

int loadPointsToMemory( string fileName, Matrix4d matrix ) {
	ifstream input(fileName.c_str());
	string line;
	double x, y, z;
	int num;

	if (!input)
		printf("Unable to open: %s\n", fileName.c_str());
	else {
		int beginIndex = points.size(),nrVertex;
		while (getline(input, line)) {
			sscanf(line.c_str(), "%lf;%lf;%lf", &x, &y, &z);

			Vector4d coord;
			coord << x, y, z, 1;

			coord = matrix * coord;
			points.push_back(Point(coord(0), coord(1), coord(2)));
		}
		nrVertex = points.size() - beginIndex;
		return nrVertex;

	}
	return 0;
}

void loadPointsToRotate(string fileName){
    ifstream input(fileName.c_str());
    string line;
    shared_ptr<Rotate> rotateInstance = rotates.back();

    vector<float> * rotatePoints = rotateInstance->getModelPoints();
    double x, y, z;
    int num;
    int pos = 0;

    if (!input)
        printf("Unable to open: %s\n", fileName.c_str());
    else {
        while (getline(input, line)) {
            sscanf(line.c_str(), "%lf;%lf;%lf", &x, &y, &z);
            rotatePoints->push_back(x);
            rotatePoints->push_back(y);
            rotatePoints->push_back(z);;
        }
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

	//TODO: se for passado só x ou só y ou z, é considerado sintace válida, corrigir.
	void createNewRotate(TiXmlElement *elem, Matrix4d matrix) {
		double x, y, z;
		int time;
        elem->Attribute("axisX", &x);
        elem->Attribute("axisY", &y);
        elem->Attribute("axisZ", &z);
		elem->Attribute("time", &time);
        shared_ptr<Point> pivot (new Point(x,y,z));

        shared_ptr<Rotate> newRotate (new Rotate(time,pivot,matrix));
		rotates.push_back(newRotate);
		nrOfRotates++;
	}

	void setDrawSegmentValues(DrawSegment * segment,TiXmlElement * model){
		double paramValue;
		if(model->Attribute("diffR",&paramValue)){
			segment->setDiffR((float)paramValue);
		}
		if(model->Attribute("diffG",&paramValue)){
			segment->setDiffG((float)paramValue);
		}
		if(model->Attribute("diffB",&paramValue)){
			segment->setDiffB((float)paramValue);
		}

		if(model->Attribute("ambR",&paramValue)){
			segment->setAmbR((float)paramValue);
		}
		if(model->Attribute("ambG",&paramValue)){
			segment->setAmbG((float)paramValue);
		}
		if(model->Attribute("ambB",&paramValue)){
			segment->setAmbB((float)paramValue);
		}

		if(model->Attribute("specR",&paramValue)){
			segment->setSpecR((float)paramValue);
		}
		if(model->Attribute("specG",&paramValue)){
			segment->setSpecG((float)paramValue);
		}
		if(model->Attribute("specB",&paramValue)){
			segment->setSpecB((float)paramValue);
		}

		if(model->Attribute("emR",&paramValue)){
			segment->setEmR((float)paramValue);
		}
		if(model->Attribute("emG",&paramValue)){
			segment->setEmG((float)paramValue);
		}
		if(model->Attribute("emB",&paramValue)){
			segment->setEmB((float)paramValue);
		}
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
                    createNewRotate(elem,matrix);
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
                	int beginIndex = points.size(), nrVertex;
                    nrVertex = loadPointsToMemory(elem->Attribute("file"), matrix);
                    DrawSegment segment = DrawSegment(beginIndex, nrVertex);
                    setDrawSegmentValues(&segment,elem);
                    segments.push_back(segment);
                    
                }
            }
            else if(specialFlag == 1){
            	for (elem = elem->FirstChildElement("model"); elem != NULL; elem = elem->NextSiblingElement("model")) {
                	loadPointsToRotate(elem->Attribute("file"));
                }
                //specialFlag = 0;
            }
		}

		// Other groups
		for (elem = group->FirstChildElement("group"); elem != NULL; elem = elem->NextSiblingElement("group"))
			loadGroupFromXML(elem, matrix);

	}

	void registerLight(TiXmlElement * light){
		double X=0,Y=0, Z=0;
		const char * type = NULL;

		type = light->Attribute("type");
		light->Attribute("X",&X);
		light->Attribute("Y",&Y);
		light->Attribute("Z",&Z);

		if(type != NULL){
			if(!strcmp(type, "POINT")){
				posOrDirOfLights.push_back(X);
				posOrDirOfLights.push_back(Y);
				posOrDirOfLights.push_back(Z);
				posOrDirOfLights.push_back(1.0);

				nrOfLights++;
			}
			else if(!strcmp(type, "DIRECTIONAL")){
				posOrDirOfLights.push_back(X);
				posOrDirOfLights.push_back(Y);
				posOrDirOfLights.push_back(Z);
				posOrDirOfLights.push_back(0.0);

				nrOfLights++;
			}
		}


	}
	

	void loadXML(const char *xmlFile) {
		TiXmlDocument doc(xmlFile);
		Matrix4d matrix = Matrix4d::Identity(4, 4);

		bool loaded = doc.LoadFile();
		if (loaded) {
			TiXmlElement * root = doc.FirstChildElement("scene");
			if(root){
				TiXmlElement * lights = root->FirstChildElement("lights");
				if(lights){
					for(TiXmlElement * light = lights->FirstChildElement("light");
						light != NULL && nrOfLights < 8;light = light -> NextSiblingElement("light")){
						registerLight(light);
					}
				}
				for (TiXmlElement *group = root->FirstChildElement("group");
					 group != NULL; group = group->NextSiblingElement("group"))
					loadGroupFromXML(group, matrix);
			}
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

	Matrix4d rotateMatrixwtime(double x, double y, double z, Matrix4d matrix, int time) {
		double angle, angleCosine, angleSin, timeNow;
		Matrix4d rotationMatrix(4, 4);

		timeNow = glutGet(GLUT_ELAPSED_TIME);
		angle = timeNow * ((2 * M_PI) / (time * 1000));

		angleCosine = cos(angle);
		angleSin = sin(angle);

		rotationMatrix << (x * x * (1 - angleCosine) + angleCosine), (x * y * (1 - angleCosine) - z * angleSin), (
				x * z * (1 - angleCosine) + y * angleSin), 0,
				(y * x * (1 - angleCosine) + z * angleSin), (y * y * (1 - angleCosine) + angleCosine), (
				y * z * (1 - angleCosine) - x * angleSin), 0,
				(z * x * (1 - angleCosine) - y * angleSin), (z * y * (1 - angleCosine) + x * angleSin), (
				z * z * (1 - angleCosine) + angleCosine), 0,
				0, 0, 0, 1;

		return rotationMatrix * matrix;

	}

	void getRotatePointPosition(double x, double y, double z, int time, Matrix4d matrixBeforeRotation, float * pivot,float* actualPoint){
		Vector4d coord;
		Matrix4d matrix = rotateMatrixwtime(pivot[0],pivot[1],pivot[2],matrixBeforeRotation,time);

        coord << x, y, z, 1;

        coord = matrix * coord;
        actualPoint[0] = coord(0);
        actualPoint[1] = coord(1);
        actualPoint[2] = coord(2);

	}

	void fillChangingNormalsVBO(){
		normalv = (float *) malloc(sizeof(float) * changingPoints.size());
		float v1[3],v2[3];
		float n[3],norm;

		for(int i = 0; i < changingPoints.size() / 9; i++){
			v1[0] = changingPoints.at(i*9+3) - changingPoints.at(i*9+0);
			v1[1] = changingPoints.at(i*9+4) - changingPoints.at(i*9+1);
			v1[2] = changingPoints.at(i*9+5) - changingPoints.at(i*9+2);
			v2[0] = changingPoints.at(i*9+6) - changingPoints.at(i*9+0);
			v2[1] = changingPoints.at(i*9+7) - changingPoints.at(i*9+1);
			v2[2] = changingPoints.at(i*9+8) - changingPoints.at(i*9+2);

			n[0] = v1[1]*v2[2] - v1[2]*v2[1];
			n[1] = v1[2]*v2[0] - v1[0]*v2[2];
			n[2] = v1[0]*v2[1] - v1[1]*v2[0];

			norm = sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));

			n[0] = n[0] / norm;
			n[1] = n[1] / norm;
			n[2] = n[2] / norm;

			normalv[i*9+0] = n[0];
			normalv[i*9+1] = n[1];
			normalv[i*9+2] = n[2];
			normalv[i*9+3] = n[0];
			normalv[i*9+4] = n[1];
			normalv[i*9+5] = n[2];
			normalv[i*9+6] = n[0];
			normalv[i*9+7] = n[1];
			normalv[i*9+8] = n[2];
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffers[3]);;
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * changingPoints.size(), normalv, GL_STATIC_DRAW);
	}

	void drawDynamicRotates(){
		vector<shared_ptr<Rotate> >::iterator it;
		vector<float> * modelPoints;
		Matrix4d matrix;
		int pos, time, r,i;
		float * pivot;
		float * point = (float*)malloc(sizeof(float) * 3);


		for (r = 0,it = rotates.begin(); it != rotates.end(); it++,r++){
			modelPoints = rotates.at(r)->getModelPoints();
			time = rotates.at(r)->getTime();
			matrix = rotates.at(r)->getMatrix();
			pivot = rotates.at(r)->getPivot();
			int nrOfPoints = modelPoints->size() / 3;
			for (i = 0; i < nrOfPoints; i+=1){
				getRotatePointPosition(modelPoints->at(i*3),modelPoints->at(i * 3+1),modelPoints->at(i * 3+2),time,matrix,pivot,point);
				changingPoints.push_back(point[0]);
				changingPoints.push_back(point[1]);
				changingPoints.push_back(point[2]);
			}
		}
		fillChangingNormalsVBO();

		glBindBuffer(GL_ARRAY_BUFFER,buffers[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * changingPoints.size(), &(changingPoints[0]), GL_STATIC_DRAW);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER,buffers[3]);
		glNormalPointer(GL_FLOAT, 0, 0);
		glDrawArrays(GL_TRIANGLES,0,changingPoints.size()/3);
		changingPoints.clear();

	}


	//TODO: sun should be emissive(plus light at center of sun = pretty good looking solar system)
	void renderScene(void) {
		glPolygonMode(GL_FRONT, mode);
		// clear buffers
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// set the camera
		glLoadIdentity();
		gluLookAt(250.0 * cos(rotateVertical) * sin(rotateHorizontal), 250.0 * sin(rotateVertical),
				  250.0 * cos(rotateVertical) * cos(rotateHorizontal),
				  0.0, 0.0, 0.0,
				  0.0f, 1.0f, 0.0f);

		float amb[4] = {0.2, 0.2, 0.2, 1.0};
		float diff[4] = {1.0, 1.0, 1.0, 1.0};
		float pos[4];
		for(int i = 0; i < nrOfLights; i++){
			pos[0] = posOrDirOfLights.at(i*4+0);
			pos[1] = posOrDirOfLights.at(i*4+1);
			pos[2] = posOrDirOfLights.at(i*4+2);
			pos[3] = posOrDirOfLights.at(i*4+3);

			//DEBUG
			//printf("lightID: %d\n\tposition: %f,%f,%f,%f\n",GL_LIGHT0+i,posOrDirOfLights.at(i*4+0),posOrDirOfLights.at(i*4+1),posOrDirOfLights.at(i*4+2),posOrDirOfLights.at(i*4+3));

			glLightfv(GL_LIGHT0+i, GL_POSITION, pos);
			glLightfv(GL_LIGHT0+i, GL_AMBIENT, amb);
			glLightfv(GL_LIGHT0+i, GL_DIFFUSE, diff);
		}

		drawDynamicRotates();
		//drawDynamicTranslates

		glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
		glNormalPointer(GL_FLOAT, 0, 0);


		float spec[4], em[4];
		diff[3] = 1.0; amb[3] = 1.0; spec[3] = 1.0; em[3] = 1.0;
		int beginIndex, nrVertex;
		vector<DrawSegment>::iterator it;
		for(it = segments.begin(); it != segments.end(); it++){
			diff[0] = it->getDiffR(); diff[1] = it->getDiffG(); diff[2] = it->getDiffB();
			//DEBUG
			//printf("diff: %f, %f, %f, %f\n",diff[0],diff[1],diff[2],diff[3]);
			glMaterialfv(GL_FRONT,GL_DIFFUSE,diff);

			amb[0] = it->getAmbR(); amb[1] = it->getAmbG(); amb[2] = it->getAmbB();
			glMaterialfv(GL_FRONT,GL_AMBIENT,amb);

			spec[0] = it->getSpecR(); spec[1] = it->getSpecG(); spec[2] = it->getSpecB();
			glMaterialfv(GL_FRONT,GL_SPECULAR,spec);

			em[0] = it->getEmR(); em[1] = it->getEmG(); em[2] = it->getEmB();
			glMaterialfv(GL_FRONT,GL_EMISSION,em);

			beginIndex = it->getBeginIndex(); nrVertex = it->getNrVertex();
			glDrawArrays(GL_TRIANGLES, beginIndex, nrVertex);
		}
		

		

		// End of frame
		glutSwapBuffers();


	}

	void fillConstantNormalsVBO(){
		normalb = (float *) malloc(sizeof(float) * points.size() * 3);
		float v1[3],v2[3];
		float n[3],norm;

		for(int i = 0; i < points.size() / 3; i++){
			v1[0] = vertexb[i*9+3] - vertexb[i*9+0];
			v1[1] = vertexb[i*9+4] - vertexb[i*9+1];
			v1[2] = vertexb[i*9+5] - vertexb[i*9+2];
			v2[0] = vertexb[i*9+6] - vertexb[i*9+0];
			v2[1] = vertexb[i*9+7] - vertexb[i*9+1];
			v2[2] = vertexb[i*9+8] - vertexb[i*9+2];

			n[0] = v1[1]*v2[2] - v1[2]*v2[1];
			n[1] = v1[2]*v2[0] - v1[0]*v2[2];
			n[2] = v1[0]*v2[1] - v1[1]*v2[0];

			norm = sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));

			n[0] = n[0] / norm;
			n[1] = n[1] / norm;
			n[2] = n[2] / norm;

			normalb[i*9+0] = n[0];
			normalb[i*9+1] = n[1];
			normalb[i*9+2] = n[2];
			normalb[i*9+3] = n[0];
			normalb[i*9+4] = n[1];
			normalb[i*9+5] = n[2];
			normalb[i*9+6] = n[0];
			normalb[i*9+7] = n[1];
			normalb[i*9+8] = n[2];
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * points.size() * 3, normalb, GL_STATIC_DRAW);
	}

	void fillConstantVBOs() {

		vector<Point>::iterator it;
		glColor3f(color1, color2, color3);
		vertexb = (float *) malloc(sizeof(float) * points.size() * 3);
		
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

		
		glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);;
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * points.size() * 3, vertexb, GL_STATIC_DRAW);

		fillConstantNormalsVBO();

	}
	void initGL() {
		//  OpenGL settings
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glGenBuffers(4, buffers);

		//DEBUG
		float red[4] = {0.8f, 0.2f, 0.2f, 1.0f};
		glMaterialfv(GL_FRONT, GL_DIFFUSE, red);

		

		if(nrOfLights > 0)
			glEnable(GL_LIGHTING);

		for(int i = 0; i < nrOfLights; i++){
			glEnable(GL_LIGHT0+i);
		}

		


		

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
		glutIdleFunc(renderScene);
		glutReshapeFunc(changeSize);

		// Callback registration for keyboard processing
		glutKeyboardFunc(handleKeyboardEvent);
		glutSpecialFunc(handleKeyboardSpecialEvent);

#ifndef __APPLE__
		//initialize glew
		glewInit();
#endif


		//Load all elements from xml file
		loadXML(argv[1]);

		for(int i = 0; i < nrOfLights; i++){
			printf("Light %d: (%f,%f,%f,%f)\n",i,posOrDirOfLights.at(i*4+0),posOrDirOfLights.at(i*4+1),posOrDirOfLights.at(i*4+2),posOrDirOfLights.at(i*4+3));
		}

		//Initialize OpenGL
		initGL();

		//Fill buffer[0] and buffer[2] with static vectors and respective normals
		fillConstantVBOs();

		// enter GLUT's main cycle
		glutMainLoop();

		return 1;
	}
