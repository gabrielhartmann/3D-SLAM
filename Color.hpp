/* 
 * File:   Color.hpp
 * Author: gabe
 *
 * Created on 11 April 2012, 3:56 PM
 */

#ifndef COLOR_HPP
#define	COLOR_HPP

#include <GL/gl.h>

class Color
{
public:

    static void setColor(double r, double g, double b); 
private:
    // material properties
    static GLfloat mat_specular[4];
    static GLfloat mat_ambient_and_diffuse[4];
    static GLfloat mat_shininess[1];
};


#endif	/* COLOR_HPP */

