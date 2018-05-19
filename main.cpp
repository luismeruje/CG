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


#define ROTATE 0
#define TRANSLATE 1
#define SCALE 2
#define ROTATE_TIME 3
#define TRANSLATE_TIME 4
#define TransformationType int

using namespace std;

float rotateHorizontal = 0;
float rotateVertical = 0;
int mode = GL_LINE;

GLuint buffers[2];


//Four doubles per light
vector<double> posOrDirOfLights;
float ambLight[4] = {0.2, 0.2, 0.2, 1.0};
float diffLight[4] = {1.0, 1.0, 1.0, 1.0};
float posLight[4];
int nrOfLights = 0;



//WARNING: Princípios de encapsulamento não foram aplicados por ser um programa simples e para aumentar performance. 
//São passadas referências de objetos entre funções e não cópias.
//TODO: destroy method's

void calculateNormals(vector<float> * vertexb, vector<float> * normalb){
	normalb->clear();
	float v1[3],v2[3];
	float n[3],norm;

	for(int i = 0; i < vertexb->size() / 9; i++){
		v1[0] = vertexb->at(i*9+3) - vertexb->at(i*9+0);
		v1[1] = vertexb->at(i*9+4) - vertexb->at(i*9+1);
		v1[2] = vertexb->at(i*9+5) - vertexb->at(i*9+2);
		v2[0] = vertexb->at(i*9+6) - vertexb->at(i*9+0);
		v2[1] = vertexb->at(i*9+7) - vertexb->at(i*9+1);
		v2[2] = vertexb->at(i*9+8) - vertexb->at(i*9+2);

		n[0] = v1[1]*v2[2] - v1[2]*v2[1];
		n[1] = v1[2]*v2[0] - v1[0]*v2[2];
		n[2] = v1[0]*v2[1] - v1[1]*v2[0];

		norm = sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));

		n[0] = n[0] / norm;
		n[1] = n[1] / norm;
		n[2] = n[2] / norm;

		normalb->push_back(n[0]);
		normalb->push_back(n[1]);
		normalb->push_back(n[2]);
		normalb->push_back(n[0]);
		normalb->push_back(n[1]);
		normalb->push_back(n[2]);
		normalb->push_back(n[0]);
		normalb->push_back(n[1]);
		normalb->push_back(n[2]);
	}
}

class MaterialLightProperties
{
	float diffR, diffG, diffB;
	float ambR, ambG, ambB;
	float specR, specG, specB;
	float emR, emG, emB;

public:
	MaterialLightProperties(){
		//DEFAULT values

		//Almost white
		this->ambR=0.2; this->ambG=0.2; this->ambB=0.2;
		
		//Almost black
		this->diffR=0.8; this->diffG=0.8; this->diffB=0.8;

		//Black(no effect)
		this->specR=0; this->specG=0; this->specB=0;

		//Black(no effect)
		this->emR=0; this->emG=0; this->emB=0;

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
};




class Model{
	vector<float> vertexb;
	vector<float> normalb;
	MaterialLightProperties lightProperties;

public:
	Model(MaterialLightProperties lightProperties){
		//Bad idea to pass vector of points as argument,
		//Could imply making copy of all coordinates.

		this->lightProperties = lightProperties;

	}

	vector<float> * getVertexb(){return &vertexb;}

	void draw(){
		calculateNormals(&vertexb,&normalb);
		

		float spec[4], em[4],diff[4],amb[4];
		diff[3] = 1.0; amb[3] = 1.0; spec[3] = 1.0; em[3] = 1.0;
		diff[0] = lightProperties.getDiffR(); diff[1] = lightProperties.getDiffG(); diff[2] = lightProperties.getDiffB();
		//DEBUG
		//printf("diff: %f, %f, %f, %f\n",diff[0],diff[1],diff[2],diff[3]);
		glMaterialfv(GL_FRONT,GL_DIFFUSE,diff);

		amb[0] = lightProperties.getAmbR(); amb[1] = lightProperties.getAmbG(); amb[2] = lightProperties.getAmbB();
		glMaterialfv(GL_FRONT,GL_AMBIENT,amb);

		spec[0] = lightProperties.getSpecR(); spec[1] = lightProperties.getSpecG(); spec[2] = lightProperties.getSpecB();
		glMaterialfv(GL_FRONT,GL_SPECULAR,spec);

		em[0] = lightProperties.getEmR(); em[1] = lightProperties.getEmG(); em[2] = lightProperties.getEmB();
		glMaterialfv(GL_FRONT,GL_EMISSION,em);


		glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
		glBufferData(GL_ARRAY_BUFFER,sizeof(float) * vertexb.size(),&(vertexb[0]),GL_STATIC_DRAW);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
		glBufferData(GL_ARRAY_BUFFER,sizeof(float) * normalb.size(),&(normalb[0]),GL_STATIC_DRAW);
		glNormalPointer(GL_FLOAT, 0, 0);

		glDrawArrays(GL_TRIANGLES, 0, vertexb.size()/3);

	}
};

class Transformation{
	TransformationType type;
	float x, y, z, angle, time;
	vector<float> curvePoints;

public:
	Transformation(TransformationType type){
		this->type = type;
		x = 0;
		y = 0;
		z = 0;
		angle = 0;
		time = 0;
	}

	void setX(float x){this->x = x;}
	void setY(float y){this->y = y;}
	void setZ(float z){this->z = z;}
	void setAngle(float angle){this->angle = angle;}
	void setTime(float time){this->time = time;}

	vector<float> * getCurvePoints(){return &curvePoints;}

	float getX(){return x;}
	float getY(){return y;}
	float getZ(){return z;}
	float getAngle(){return angle;}
	float getTime(){return time;}
	TransformationType getType(){return type;}

	void apply(){
		switch(type){
			case ROTATE:
				glRotatef(angle,x,y,z);
				break;
			case TRANSLATE:
				glTranslatef(x,y,z);
				break;
			case SCALE:
				glScalef(x,y,z);
				break;
			case ROTATE_TIME:

				float timeNow = glutGet(GLUT_ELAPSED_TIME);
				angle = timeNow * ((2 * M_PI) / (time * 1000));
				glRotatef(angle,x,y,z);
				break;
		}
	}

};

//TODO: group models with no time tranformations in separate class, at the end of processing, with it's own
//VBO's, to avoid having to fill identical VBO's in each frame (vs. just one time);
class Group{
	vector<Transformation> transformations;
	vector<Model> models;
	vector<Group *> subGroups;

public:
	Group(){}

	vector<Transformation> * getTransformations(){return &transformations;}
	vector<Model> * getModels(){return &models;}
	vector<Group *> * getSubGroups(){return &subGroups;}

	void drawGroupAndSubGroups(){
		int i;
		glPushMatrix();
		vector<Transformation>::iterator transformationsIt;
		for(transformationsIt = transformations.begin();transformationsIt != transformations.end();transformationsIt++){
			transformationsIt->apply();
		}

		vector<Model>::iterator modelsIt;
		for(modelsIt = models.begin(); modelsIt != models.end(); modelsIt++){
			modelsIt->draw();
		}

		vector<Group *>::iterator subGroupsIt;
		for(i = 0,subGroupsIt = subGroups.begin(); subGroupsIt != subGroups.end(); subGroupsIt++,i++){	
			subGroups.at(i)->drawGroupAndSubGroups();
			
		}
		glPopMatrix();
	}

	void addTransformation(Transformation t){
		transformations.push_back(t);
	}
	
};


vector<Group *> rootGroups;



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

	void setMaterialLightPropertiesValues(MaterialLightProperties * properties,TiXmlElement * model){
		double paramValue;
		if(model->Attribute("diffR",&paramValue)){
			properties->setDiffR((float)paramValue);
		}
		if(model->Attribute("diffG",&paramValue)){
			properties->setDiffG((float)paramValue);
		}
		if(model->Attribute("diffB",&paramValue)){
			properties->setDiffB((float)paramValue);
		}

		if(model->Attribute("ambR",&paramValue)){
			properties->setAmbR((float)paramValue);
		}
		if(model->Attribute("ambG",&paramValue)){
			properties->setAmbG((float)paramValue);
		}
		if(model->Attribute("ambB",&paramValue)){
			properties->setAmbB((float)paramValue);
		}

		if(model->Attribute("specR",&paramValue)){
			properties->setSpecR((float)paramValue);
		}
		if(model->Attribute("specG",&paramValue)){
			properties->setSpecG((float)paramValue);
		}
		if(model->Attribute("specB",&paramValue)){
			properties->setSpecB((float)paramValue);
		}

		if(model->Attribute("emR",&paramValue)){
			properties->setEmR((float)paramValue);
		}
		if(model->Attribute("emG",&paramValue)){
			properties->setEmG((float)paramValue);
		}
		if(model->Attribute("emB",&paramValue)){
			properties->setEmB((float)paramValue);
		}
	}

	void addModel(TiXmlElement * elem, vector<Model> * models){
		string fileName = elem->Attribute("file");

		MaterialLightProperties properties = MaterialLightProperties();

		setMaterialLightPropertiesValues(&properties,elem);

		Model model = Model(properties);
		vector<float> * vertexb = model.getVertexb();


		ifstream input(fileName.c_str());
		if (!input)
			printf("Unable to open: %s\n", fileName.c_str());
		else {
			double x, y, z;
			string line;
			while (getline(input, line)) {
				sscanf(line.c_str(), "%lf;%lf;%lf", &x, &y, &z);
				vertexb->push_back(x);
				vertexb->push_back(y);
				vertexb->push_back(z);
			}

		}
		models->push_back(model);
	}

	void addTransformation(TransformationType type,TiXmlElement * elem,Group * currentGroup){
		Transformation t = Transformation(type);
		double x=0,y=0,z=0,time=0,angle=0;

		elem->Attribute("X",&x);
		elem->Attribute("Y",&y);
		elem->Attribute("Z",&z);

		switch(type){
			case ROTATE:
				elem->Attribute("axisX",&x);
				elem->Attribute("axisY",&y);
				elem->Attribute("axisZ",&z);
				elem->Attribute("angle",&angle);
				break;
			case ROTATE_TIME:
				printf("One rotateTime");
				elem->Attribute("axisX",&x);
				elem->Attribute("axisY",&y);
				elem->Attribute("axisZ",&z);
				elem->Attribute("time",&time);
				break;
		}

		t.setX((float)x);
		t.setY((float)y);
		t.setZ((float)z);
		t.setTime((float)time);
		t.setAngle((float)angle);
		currentGroup->addTransformation(t);

	}

	void loadGroupFromXML(TiXmlElement *groupElem, Group * currentGroup) {
		vector<Group *> * subGroups = currentGroup->getSubGroups();
		vector<Transformation> * transformations = currentGroup->getTransformations();
		vector<Model> * models = currentGroup->getModels();

		TiXmlElement * elem;
		double angle, x, y, z;
		int translateFlag = 0, rotateFlag = 0, scaleFlag = 0;


		for (elem = groupElem->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement()) {
			if (strcmp(elem->Value(), "translate") == 0 && translateFlag == 0) {
				addTransformation(TRANSLATE,elem,currentGroup);
				translateFlag++;
			} else if (strcmp(elem->Value(), "rotate") == 0 && rotateFlag == 0) {
                const char *isSpecialRotate = elem->Attribute("time");
				if (isSpecialRotate) {
                    addTransformation(ROTATE_TIME,elem,currentGroup);
                }
				else{
					addTransformation(ROTATE,elem,currentGroup);
				}
				rotateFlag++;
			} else if (strcmp(elem->Value(), "scale") == 0 && scaleFlag == 0) {
				addTransformation(SCALE,elem,currentGroup);
				scaleFlag++;
			}
		}

		// Models
		elem = groupElem->FirstChildElement("models");
		if (elem != NULL) {
                for (elem = elem->FirstChildElement("model"); elem != NULL; elem = elem->NextSiblingElement("model")) {
                    addModel(elem, models);
                    //DrawSegment segment = DrawSegment(beginIndex, nrVertex);
                    //setDrawSegmentValues(&segment,elem);
                    //staticPointsSegments.push_back(segment);
                }
            
		}

		// Other groups
		for (elem = groupElem->FirstChildElement("group"); elem != NULL; elem = elem->NextSiblingElement("group")){
			Group * subGroup = new Group();
			subGroups->push_back(subGroup);
			loadGroupFromXML(elem, subGroup);
		}

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
				for (TiXmlElement *groupElem = root->FirstChildElement("group");
					 groupElem != NULL; groupElem = groupElem->NextSiblingElement("group")){
					
					Group * rootGroup = new Group();
					rootGroups.push_back(rootGroup);
					loadGroupFromXML(groupElem,rootGroup);
				}
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
			default:
				break;
		}
		glutPostRedisplay();
	}

	void renderScene(void) {
		int i;

		glPolygonMode(GL_FRONT, mode);
		// clear buffers
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// set the camera
		glLoadIdentity();
		gluLookAt(150.0 * cos(rotateVertical) * sin(rotateHorizontal), 150.0 * sin(rotateVertical),
				  150.0 * cos(rotateVertical) * cos(rotateHorizontal),
				  0.0, 0.0, 0.0,
				  0.0f, 1.0f, 0.0f);

		
		for(i = 0; i < nrOfLights; i++){
			posLight[0] = posOrDirOfLights.at(i*4+0);
			posLight[1] = posOrDirOfLights.at(i*4+1);
			posLight[2] = posOrDirOfLights.at(i*4+2);
			posLight[3] = posOrDirOfLights.at(i*4+3);

			//DEBUG
			//printf("lightID: %d\n\tposition: %f,%f,%f,%f\n",GL_LIGHT0+i,posOrDirOfLights.at(i*4+0),posOrDirOfLights.at(i*4+1),posOrDirOfLights.at(i*4+2),posOrDirOfLights.at(i*4+3));

			glLightfv(GL_LIGHT0+i, GL_POSITION, posLight);
			glLightfv(GL_LIGHT0+i, GL_AMBIENT, ambLight);
			glLightfv(GL_LIGHT0+i, GL_DIFFUSE, diffLight);
		}		
		glColor3f(1.0, 1.0, 0.5);
		vector<Group *>::iterator rootGroupsIt;

		for(i=0,rootGroupsIt = rootGroups.begin(); rootGroupsIt != rootGroups.end(); rootGroupsIt++,i++){
			rootGroups.at(i)->drawGroupAndSubGroups();
		}

		// End of frame
		glutSwapBuffers();


	}


	void initGL() {
		//  OpenGL settings
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glGenBuffers(2, buffers);

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

		// enter GLUT's main cycle
		glutMainLoop();

		return 1;
	}