// Lighting.cpp: implementation of the CLighting and CLight class.
// Copyright: (c) Burkhard Wuensche, 1998
// This file contains subset of the original lighting class (c) 2002
//////////////////////////////////////////////////////////////////////

#include "Lighting.h"

CLight::CLight(int id):glLightID(id)
{
	light_position[0]=0.0f;
	light_position[1]=0.0f;
	light_position[2]=1.0f;
	light_position[3]=1.0f;
	light_ambient[0]=0.4f;
	light_ambient[1]=0.4f;
	light_ambient[2]=0.4f;
	light_ambient[3]=1.0f;
	light_diffuse[0]=0.6f;
	light_diffuse[1]=0.6f;
	light_diffuse[2]=0.6f;
	light_diffuse[3]=1.0f;
	light_specular[0]=1.0f;
	light_specular[1]=1.0f;
	light_specular[2]=1.0f;
	light_specular[3]=1.0f;
}

void CLight::init()
{ 
	glLightfv(glLightID, GL_SPECULAR, light_specular);
	glLightfv(glLightID, GL_AMBIENT, light_ambient);
	glLightfv(glLightID, GL_DIFFUSE, light_diffuse);
	glLightfv(glLightID, GL_POSITION, light_position);
}

void CLight::setPosition(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
	light_position[0]=x;
	light_position[1]=y;
	light_position[2]=z;
	light_position[3]=w;
	init();
}


CLighting::CLighting()
{
	numLights=1; 
	lights = new CLight*[numLights];
	lights[0] = new CLight(GL_LIGHT0);
                lights[0]->setPosition(50.0, 200.0, 10.0, 1.0f);
//	lights[0]->setPosition(0,0,50.0f,1.0f);
//	lights[0]->setPosition(30.0f,30.0f,50.0f,1.0f);
}

void CLighting::enable() const
{ 
	glEnable(GL_LIGHTING);
	for(int i=0;i<numLights;i++) glEnable(lights[i]->glLightID);
}

void CLighting::disable() const
{
	glDisable(GL_LIGHTING);
	for(int i=0;i<numLights;i++) glDisable(lights[i]->glLightID);
}

void CLighting::init() const
{ 
	for(int i=0;i<numLights;i++) lights[i]->init();
}
