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
#define TEXTURE_SPHERE 1

using namespace std;

float rotateHorizontal = -0.2;
float rotateVertical = 0.5;
int mode = GL_FILL;
float cameraDistance = 150;

GLuint buffers[3];


//Four doubles per light
vector<double> posOrDirOfLights;
float ambLight[4] = {0.5, 0.5, 0.5, 1.0};
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

void calculateTextureCoordinates(vector<float> * vertexb, vector<float> * textureb, int stacks, int slices){
	float x, next_x, y, next_y, stack, slice;
	for (slice = 0; slice < slices; slice++)
		if (slice+1 != slices){
			textureb->push_back(0.5); textureb->push_back(0.0);
			textureb->push_back((1/(float)slices)*slice); textureb->push_back(0.0); 
			textureb->push_back((1/(float)slices)*(slice+1));textureb->push_back(0.0);
		}else{
			textureb->push_back(0.5); textureb->push_back(0.0);
			textureb->push_back((1/(float)slices)*(float)slice); textureb->push_back(0.0); 
			textureb->push_back(1); textureb->push_back(0.0);
		}
	//Nested Loops para os calculos dos triangulos entre stacks em que nenhuma e um dos polos da esfera
	if(stacks > 2){
		for(stack = 1; stack < stacks; stack+= 1){
			y = (1/(float)stacks) * stack;
			next_y = (1/(float)stacks) * (stack+1);
			for(slice = 0; slice < slices; slice += 1){
				x = (1/(float)slices) * slice;
				if (slice + 1 == slices){
					next_x = 0;
				}
				else{
					next_x = (1/(float)slices) * (slice + 1);
				}
				//1st triangle
				textureb->push_back(x);textureb->push_back(y);
				textureb->push_back(next_x);textureb->push_back(next_y);
				textureb->push_back(next_x);textureb->push_back(y);
				//2nd triangle
				textureb->push_back(x);textureb->push_back(y);
				textureb->push_back(x);textureb->push_back(next_y);
				textureb->push_back(next_x);textureb->push_back(next_y);
			}
		}
	}

	//Entre ultima stack e polo y = +radius
	for (x = 0; x < slices; x+= 1){
		if (x+1 != slices){
			textureb->push_back(0.5);textureb->push_back(1);
			textureb->push_back((1/(float)slices)*(float)(x+1));textureb->push_back(1);
			textureb->push_back((1/(float)slices)*(float)x);textureb->push_back(1);
		}
		else {
			textureb->push_back(0.5);textureb->push_back(1);
			textureb->push_back(1);textureb->push_back(1);
			textureb->push_back((1/(float)slices)*(float)x);textureb->push_back(1);
		}
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
	vector<float> textureb;
	int textureID;
	int textureType; 
	//Considering that textures are always of sphere
	//TODO: create class texture?
	int stacks;
	int slices;
	MaterialLightProperties lightProperties;

public:
	Model(MaterialLightProperties lightProperties){

		this->lightProperties = lightProperties;
		this->textureID = -1;
		this->textureType = -1;
		this->stacks = -1;
		this->slices = -1;
	}

	vector<float> * getVertexb(){return &vertexb;}

	void setTextureID(int textureID){this->textureID = textureID;}
	void setTextureType(int textureType){this->textureType = textureType;}
	void setStacks(int stacks){this->stacks = stacks;}
	void setSlices(int slices){this->slices = slices;}

	void draw(){
		if(normalb.size()== 0){
			calculateNormals(&vertexb,&normalb);
		}

		if(textureb.size()==0 && textureID != -1){
			calculateTextureCoordinates(&vertexb,&textureb,stacks,slices);
		}
		


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

		if(textureb.size() == 0){
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glTexCoordPointer(2,GL_FLOAT,0,0);
		}
		else{
			glBindTexture(GL_TEXTURE_2D,textureID);
			glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
			glBufferData(GL_ARRAY_BUFFER,sizeof(float) * textureb.size(),&(textureb[0]),GL_STATIC_DRAW);
			glTexCoordPointer(2,GL_FLOAT,0,0);

		}


		glDrawArrays(GL_TRIANGLES, 0, vertexb.size()/3);
		glBindTexture(GL_TEXTURE_2D,0);

	}
};

class Transformation{
	TransformationType type;
	float x, y, z, angle, time;
	vector<float> curvePoints;
	float pos[4], deriv[4], ys[3] = {0, 1, 0}, zs[3], rotMatrix[16];


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
	void savePoint(float x,float y, float z){
		curvePoints.push_back(x);
		curvePoints.push_back(y);
		curvePoints.push_back(z);
	}


	float getX(){return x;}
	float getY(){return y;}
	float getZ(){return z;}
	float getAngle(){return angle;}
	float getTime(){return time;}
	TransformationType getType(){return type;}

	void buildRotMatrix(float *x, float *y, float *z, float *m) {

		m[0] = x[0]; m[1] = x[1]; m[2] = x[2]; m[3] = 0;
		m[4] = y[0]; m[5] = y[1]; m[6] = y[2]; m[7] = 0;
		m[8] = z[0]; m[9] = z[1]; m[10] = z[2]; m[11] = 0;
		m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
	}


	void cross(float *a, float *b, float *res) {

		res[0] = a[1]*b[2] - a[2]*b[1];
		res[1] = a[2]*b[0] - a[0]*b[2];
		res[2] = a[0]*b[1] - a[1]*b[0];
	}


	void normalize(float *a) {

		float l = sqrt(a[0]*a[0] + a[1] * a[1] + a[2] * a[2]);
		a[0] = a[0]/l;
		a[1] = a[1]/l;
		a[2] = a[2]/l;
	}


	float length(float *v) {

		float res = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
		return res;

	}

	void multMatrixVector(float *m, float *v, float *res) {

		for (int j = 0; j < 4; ++j) {
			res[j] = 0;
			for (int k = 0; k < 4; ++k) {
				res[j] += v[k] * m[j * 4 + k];
			}
		}

	}

	void renderCatmullRomCurve() {

	// desenhar a curva usando segmentos de reta - GL_LINE_LOOP
		glBegin(GL_LINE_LOOP);
			float t;
			for (t = 0; t < 1; t += 0.01) {
				getGlobalCatmullRomPoint( t,pos,deriv );
				glVertex3f( pos[0],pos[1],pos[2] );

			}

		glEnd();
	}
	void getCatmullRomPoint(float t, float *p0, float *p1, float *p2, float *p3, float *pos, float *deriv) {

		// catmull-rom matrix
		float m[4][4] = {	{-0.5f,  1.5f, -1.5f,  0.5f},
							{ 1.0f, -2.5f,  2.0f, -0.5f},
							{-0.5f,  0.0f,  0.5f,  0.0f},
							{ 0.0f,  1.0f,  0.0f,  0.0f}
						};

		// Compute A = M * P

		float a[4][4];

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				a[i][j] = m[i][0] * p0[j] +
							m[i][1] * p1[j] +
							m[i][2] * p2[j] +
							m[i][3] * p3[j];
			}

		// Compute pos = T * A

		for (int i = 0; i < 4; i++)
		{
			pos[i] = a[0][i] * pow( t,3 ) +
							 a[1][i] * pow( t,2 ) +
							 a[2][i] * t +
							 a[3][i];
		}

		// compute deriv = T' * A

		for (int i = 0; i < 4; i++)
		{
			deriv[i] = a[0][i] * 3 * pow( t,2 ) +
							 	 a[1][i] * 2 * t +
							   a[2][i];
		}
	}

	void getGlobalCatmullRomPoint(float gt,float* pos, float *deriv) {

		int npoint = curvePoints.size()/3;
		float p[npoint][4];
		int j = 0;
		for (int i = 0; i < curvePoints.size(); i+=3)
		{
			p[j][0] = curvePoints.at(i);
			p[j][1] = curvePoints.at(i+1);
			p[j][2] = curvePoints.at(i+2);
			p[j][3] = 1;
			j++;
		}
		float t = gt * npoint; // this is the real global t
		int index = floor(t);  // which segment
		t = t - index; // where within  the segment

		// indices store the points
		int indices[4];
		indices[0] = (index + npoint-1)%npoint;
		indices[1] = (indices[0]+1)%npoint;
		indices[2] = (indices[1]+1)%npoint;
		indices[3] = (indices[2]+1)%npoint;

		getCatmullRomPoint(t, p[indices[0]], p[indices[1]], p[indices[2]], p[indices[3]], pos, deriv);
	}


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
			case TRANSLATE_TIME:{
				float tempo =glutGet(GLUT_ELAPSED_TIME)% (int)(time*1000);
				renderCatmullRomCurve();
				getGlobalCatmullRomPoint(time,pos,deriv);
				/*normalize( deriv );
				cross( deriv,ys,zs );
				normalize( zs );
				cross( zs,deriv,ys );
				normalize( ys );
				buildRotMatrix( deriv,ys,zs,rotMatrix );*/
				glTranslatef( pos[0],pos[1],pos[2]);
				//glMultMatrixf( rotMatrix );

				break;
			}
			case ROTATE_TIME:

				float timeNow = glutGet(GLUT_ELAPSED_TIME);
				angle = timeNow * ((360) / (time * 1000));
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

	int loadTexture(std::string s) {

		unsigned int t,tw,th;
		unsigned char *texData;
		unsigned int texID;

		ilInit();
		ilEnable(IL_ORIGIN_SET);
		ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
		ilGenImages(1,&t);
		ilBindImage(t);
		ilLoadImage((ILstring)s.c_str());
		tw = ilGetInteger(IL_IMAGE_WIDTH);
		th = ilGetInteger(IL_IMAGE_HEIGHT);
		ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);
		texData = ilGetData();

		glGenTextures(1,&texID);
		
		glBindTexture(GL_TEXTURE_2D,texID);
		glTexParameteri(GL_TEXTURE_2D,	GL_TEXTURE_WRAP_S,		GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D,	GL_TEXTURE_WRAP_T,		GL_REPEAT);

		glTexParameteri(GL_TEXTURE_2D,	GL_TEXTURE_MAG_FILTER,   	GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,	GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, texData);
		glGenerateMipmap(GL_TEXTURE_2D);

		glBindTexture(GL_TEXTURE_2D, 0);

		return texID;

	}

	void setTexture(Model * model, TiXmlElement * elem){
		int textureID = -1, stacks = -1, slices = -1;
		const char * type = NULL, * textureName = NULL;

		textureName = elem->Attribute("texture");
		type = elem->Attribute("type");
		elem->Attribute("stacks",&stacks);
		elem->Attribute("slices",&slices);

		if(stacks > 0 && slices > 0 && type != NULL){
			if(!strcmp(type,"sphere")){
				textureID = loadTexture(string(textureName));
				if(textureID != -1){
					model->setTextureID(textureID);
					model->setStacks(stacks);
					model->setSlices(slices);
					model->setTextureType(TEXTURE_SPHERE);
				}
			}
		}
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
		setTexture(&model,elem);
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

	void getTranslatePoints(TiXmlElement* elem, Transformation* t){
		double x,y,z;
		for (TiXmlElement* e = elem->FirstChildElement(); e != NULL; e = e->NextSiblingElement()) {

			if (strcmp(elem->Value(), "point") ) {
				e->Attribute("X",&x);
				e->Attribute("Y",&y);
				e->Attribute("Z",&z);
				
				t->savePoint((float) x,(float) y,(float) z);
			}
		}
	}

	void addTransformation(TransformationType type,TiXmlElement * elem,Group * currentGroup){
		Transformation t = Transformation(type);
		double x=0,y=0,z=0,time=0,angle=0;

		elem->Attribute("X",&x);
		elem->Attribute("Y",&y);
		elem->Attribute("Z",&z);

		switch(type){
			case TRANSLATE_TIME:

				elem->Attribute("time",&time);
				getTranslatePoints(elem,&t);
				break;
			case ROTATE:
				elem->Attribute("axisX",&x);
				elem->Attribute("axisY",&y);
				elem->Attribute("axisZ",&z);
				elem->Attribute("angle",&angle);
				break;
			case ROTATE_TIME:
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
				const char * isSpecialTranslate = elem->Attribute("time");
				if (isSpecialTranslate) {

                    addTransformation(TRANSLATE_TIME,elem,currentGroup);
                }
				else{
					addTransformation(TRANSLATE,elem,currentGroup);
				}
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
			case 'i':
				cameraDistance -= 2;
				break;
			case 'o':
				cameraDistance += 2;
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
		gluLookAt(cameraDistance * cos(rotateVertical) * sin(rotateHorizontal), cameraDistance * sin(rotateVertical),
				  cameraDistance * cos(rotateVertical) * cos(rotateHorizontal),
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
		glEnable(GL_TEXTURE_2D);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glGenBuffers(3, buffers);

		if(nrOfLights > 0)
			glEnable(GL_LIGHTING);

		for(int i = 0; i < nrOfLights; i++){
			glEnable(GL_LIGHT0+i);
		}
	}

	int main(int argc, char **argv) {

		printf("Use arrows to rotate image.\ni = ZOOM in;\no = ZOOM out;\nTo change filling:\nf-> FILL\nl-> LINE\np->POINT\n");

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