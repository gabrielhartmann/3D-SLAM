#include "Color.hpp"
GLfloat Color::mat_specular[4];
GLfloat Color::mat_ambient_and_diffuse[4];
GLfloat Color::mat_shininess[1];

void Color::setColor(double r, double g, double b)
{
    mat_ambient_and_diffuse[0] = r;
    mat_ambient_and_diffuse[1] = g;
    mat_ambient_and_diffuse[2] = b;
    mat_ambient_and_diffuse[3] = 1;
    mat_specular[0]=0.8f;				// ... with white highlights
    mat_specular[1]=0.8f;				// if light source is reflected
    mat_specular[2]=0.8f;				// on the material surface.
    mat_specular[3]=1;
    mat_shininess[0]=100;
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_ambient_and_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}   
