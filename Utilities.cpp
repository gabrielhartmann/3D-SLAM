// Utilities.cpp: implementation of the sveral utility functions.
//////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <GL/freeglut_std.h>
#include "Utilities.h"

////////////////////////////
// error/warning messages //
////////////////////////////

void displayMessage(MESSAGE_TYPE type, char* message, ...)
{
	va_list ap;

	va_start(ap,message);
	vsprintf(message_string,message,ap);
	switch(type){
		case ERROR_MESSAGE:{
			cerr << message_string;	
			exit(EXIT_FAILURE);
			break;
		}
		case WARNING_MESSAGE:{
			cout << message_string;	
			break;
		}
		default: displayMessage(ERROR_MESSAGE, "displayMessage(): unknown message type");
	}
	va_end(ap);
}

////////////////////////////////////
//    2D array of 3D vertices     //
//	  ( stored as float[3] )      //
////////////////////////////////////

CVertexArray2D::CVertexArray2D(){ data=NULL;}

CVertexArray2D::CVertexArray2D(int m, int n)
{
	setSize(n,m);
}

CVertexArray2D::~CVertexArray2D()
{
	int i,j;
	for(i=0;i<sizeN;i++)
	{
		for(j=0;j<sizeM;j++)
		{
			delete[] data[i][j];												
		}
		delete[] data[i];						
	}
	delete[] data;						
}

void CVertexArray2D::setSize(int n, int m)
{
	data=(float***) malloc(n*sizeof(float**));
	if (data==NULL) displayMessage(ERROR_MESSAGE, "CVertexArray2D::CVertexArray2D(n,m) could not allocate memory");
	int i,j;
	for(i=0;i<n;i++)
	{
		data[i]=(float**) malloc(m*sizeof(float*));
		if (data[i]==NULL) displayMessage(ERROR_MESSAGE, "CVertexArray2D::CVertexArray2D(n,m) could not allocate memory");
		for(j=0;j<m;j++)
		{
			data[i][j]=(float*) malloc(3*sizeof(float));							
			if (data[i][j]==NULL) displayMessage(ERROR_MESSAGE, "CVertexArray2D::CVertexArray2D(n,m) could not allocate memory");
		}
	}
	sizeN=n;
	sizeM=m;
}

float** CVertexArray2D::operator[](int row)
{
#ifdef MY_DEBUG_OPTION
	if ((row < 0) || (row >= size)) 
		display_message(ERROR_MESSAGE,"CVertexArray2D::[]. out of range: size=%d index=%d",size,index);
#endif
	return data[row];
};	

float* CVertexArray2D::operator()(int row, int column) { 
#ifdef MY_DEBUG_OPTION
	if ((row < 0) || (row >= sizeM) || (column < 0) || (column >= sizeN)) 
		display_message(ERROR_MESSAGE,"CVertexArray2D::(). out of range: size=(%d,%d), index=(%d,%d)",sizeN,sizeM,row,column);
#endif
	return data[row][column]; 
}

ostream& operator<<(ostream& s, CVertexArray2D& array)
{
	s << "\n nRows=" << array.sizeM << " nColumns=" << array.sizeN;
	for(int i=0;i<array.sizeM;i++){
		s << "\n";
		for(int j=0;j<array.sizeN;j++) 
			s << "(" << array(i,j)[0] << ", " << array(i,j)[1] << ", " <<array(i,j)[2] << "\t";
	}
	return s;
}


///////////////////////////////////
//      random numbers           //
///////////////////////////////////

/* A C-program for MT19937: Real number version  (1998/4/6)    */
/*   genrand() generates one pseudorandom real number (double) */
/* which is uniformly distributed on [0,1]-interval, for each  */
/* call. sgenrand(seed) set initial values to the working area */
/* of 624 words. Before genrand(), sgenrand(seed) must be      */
/* called once. (seed is any 32-bit integer except for 0).     */
/* Integer generator is obtained by modifying two lines.       */
/*   Coded by Takuji Nishimura, considering the suggestions by */
/* Topher Cooper and Marc Rieffel in July-Aug. 1997.           */

/* This library is free software; you can redistribute it and/or   */
/* modify it under the terms of the GNU Library General Public     */
/* License as published by the Free Software Foundation; either    */
/* version 2 of the License, or (at your option) any later         */
/* version.                                                        */
/* This library is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of  */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.            */
/* See the GNU Library General Public License for more details.    */
/* You should have received a copy of the GNU Library General      */
/* Public License along with this library; if not, write to the    */
/* Free Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA   */ 
/* 02111-1307  USA                                                 */

/* Copyright (C) 1997 Makoto Matsumoto and Takuji Nishimura.       */
/* When you use this, send an email to: matumoto@math.keio.ac.jp   */
/* with an appropriate reference to your work.                     */

/* REFERENCE                                                       */
/* M. Matsumoto and T. Nishimura,                                  */
/* "Mersenne Twister: A 623-Dimensionally Equidistributed Uniform  */
/* Pseudo-Random Number Generator",                                */
/* ACM Transactions on Modeling and Computer Simulation,           */
/* Vol. 8, No. 1, January 1998, pp 3--30.                          */

/* Period parameters */  
#define N 624
#define M 397
#define MATRIX_A 0x9908b0df   /* constant vector a */
#define UPPER_MASK 0x80000000 /* most significant w-r bits */
#define LOWER_MASK 0x7fffffff /* least significant r bits */

/* Tempering parameters */   
#define TEMPERING_MASK_B 0x9d2c5680
#define TEMPERING_MASK_C 0xefc60000
#define TEMPERING_SHIFT_U(y)  (y >> 11)
#define TEMPERING_SHIFT_S(y)  (y << 7)
#define TEMPERING_SHIFT_T(y)  (y << 15)
#define TEMPERING_SHIFT_L(y)  (y >> 18)

static unsigned long mt[N]; /* the array for the state vector  */
static int mti=N+1; /* mti==N+1 means mt[N] is not initialized */

/* initializing the array with a NONZERO seed */
void
sgenrand(unsigned long seed)
{
    /* setting initial seeds to mt[N] using         */
    /* the generator Line 25 of Table 1 in          */
    /* [KNUTH 1981, The Art of Computer Programming */
    /*    Vol. 2 (2nd Ed.), pp102]                  */
    mt[0]= seed & 0xffffffff;
    for (mti=1; mti<N; mti++)
        mt[mti] = (69069 * mt[mti-1]) & 0xffffffff;
}

unsigned long /* for integer generation */
genrand()
{
    unsigned long y;
    static unsigned long mag01[2]={0x0, MATRIX_A};
    /* mag01[x] = x * MATRIX_A  for x=0,1 */

    if (mti >= N) { /* generate N words at one time */
        int kk;

        if (mti == N+1)   /* if sgenrand() has not been called, */
            sgenrand(4357); /* a default initial seed is used   */

        for (kk=0;kk<N-M;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+M] ^ (y >> 1) ^ mag01[y & 0x1];
        }
        for (;kk<N-1;kk++) {
            y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
            mt[kk] = mt[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1];
        }
        y = (mt[N-1]&UPPER_MASK)|(mt[0]&LOWER_MASK);
        mt[N-1] = mt[M-1] ^ (y >> 1) ^ mag01[y & 0x1];

        mti = 0;
    }
  
    y = mt[mti++];
    y ^= TEMPERING_SHIFT_U(y);
    y ^= TEMPERING_SHIFT_S(y) & TEMPERING_MASK_B;
    y ^= TEMPERING_SHIFT_T(y) & TEMPERING_MASK_C;
    y ^= TEMPERING_SHIFT_L(y);

    return y; /* for integer generation */
}

double genrand2()
{
   return ( ((double) genrand()) * 2.3283064370807974e-10 ); /* reals */
}

void print(string s, Eigen::VectorXd vector)
{
    std::cout << s << " " << vector.rows() << " x " << vector.cols() << std::endl;
    std::cout << vector.transpose() << std::endl;
}

void print(string s, Eigen::Vector3d vector)
{
    std::cout << s << " " << vector.rows() << " x " << vector.cols() << std::endl;
    std::cout << vector.transpose() << std::endl;
}

void print(string s, Eigen::MatrixXd matrix)
{
    std::cout << s << " " << matrix.rows() << " x " << matrix.cols() << std::endl;
    std::cout << matrix << std::endl;
}

void clear(Eigen::VectorXd& vec)
{
    for (int row=0; row < vec.rows(); row++)
    {
        vec[row] = 0.0;
    }
}

void clear(Eigen::MatrixXd& mat)
{
    for (int row = 0; row < mat.rows(); row++)
    {
        for (int col = 0; col < mat.cols(); col++)
        {
            mat(row, col) = 0.0;
        }
    }
}

void clearCross(Eigen::MatrixXd& mat, int row, int col)
{
    // Clear row
    for (int c = 0; c < mat.cols(); c++)
    {
        mat(row, c) = 0.0;
    }
    
    // Clear column
    for (int r = 0; r < mat.rows(); r++)
    {
        mat(r, col) = 0.0;
    }
}

void identity(Eigen::MatrixXd& mat)
{
    clear(mat);
    for (int row = 0; row < mat.rows(); row++)
    {
        for (int col = 0; col < mat.cols(); col++)
        {
            if (row == col)
            {
                mat(row, col) = 1.0;    
            }
            
        }
    }
}

double sinc(double x)
{
     if (std::fabs(x) < 1.0e-4)
    {
        return (1.0) - x * x*(0.166666666666666666667);
    }
    else{
        return std::sin(x)/x;
    }
}

bool visible(Eigen::Vector3d position, Eigen::Quaterniond direction, double fov, Eigen::Vector3d lmW)
{
    Eigen::Vector3d lmC; // Landmark in Camera coordinates
    lmC = lmW - position;
    
    Eigen::Matrix3d rotMat;
    rotMat = direction;
    lmC = rotMat.transpose() * lmC;
    
    Eigen::Vector3d opticalAxis;
    opticalAxis << 0.0, 0.0, 1.0;
    
    double lengthLM = std::sqrt(lmC.adjoint() * lmC);
    double lengthOA = 1.0;
    
    double dp = lmC.adjoint() * opticalAxis;
    double theta = std::acos( dp /  (lengthLM * lengthOA));    
    
    if (theta < (fov / 2.0))
        return true;
    
    return false;
}

std::vector<int> commonTags(std::vector<Measurement> ms)
{
    // Initialize tag list
    std::vector<int> cTags = ms[0].getTags();
    
    for (int i=1; i<ms.size(); i++)
    {
        cTags = commonTags(cTags, ms[i]);
    }
    
    return cTags;
}

std::vector<int> commonTags(std::vector<int> tags, Measurement m)
{
    std::vector<int> cTags;
    
    for (int i=0; i<tags.size(); i++)
    {
        if (m.contains(tags[i]))
        {
            cTags.push_back(tags[i]);
        }
    }
    
    return cTags;
}

void drawCamera(Eigen::Vector3d pos, Eigen::Quaterniond dir, double focalLength, double r, double g, double b)
{
    glPushMatrix();
    glTranslated(pos.x(), pos.y(), pos.z());
    
    Eigen::AngleAxisd aa;
    aa = dir;
    glRotated(aa.angle() * 180.0 / PI, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
        Color::setColor(0.8, 0.8, 0.8); //white
        glVertex3d(0.0, 0.0, 0.0);

        Color::setColor(r, g, b);
        glVertex3d(-3.0, 3.0, focalLength);
        glVertex3d(3.0, 3.0, focalLength);
        glVertex3d(3.0, -3.0, focalLength);
        glVertex3d(-3.0, -3.0, focalLength);
        glVertex3d(-3.0, 3.0, focalLength);
    glEnd();
    
    glPopMatrix();
}

void drawImu(Eigen::Vector3d pos, Eigen::Quaterniond dir, double cubeSize, double r, double g, double b)
{
    glPushMatrix();
    glTranslated(pos.x(), pos.y(), pos.z());
    
    Eigen::AngleAxisd aa;
    aa = dir;
    glRotated(aa.angle() * 180.0 / PI, aa.axis().x(), aa.axis().y(), aa.axis().z());
    
    glBegin(GL_TRIANGLE_FAN);
        Color::setColor(r, g, b);
        glutSolidCube(cubeSize);
    glEnd();
    
    glPopMatrix();
}