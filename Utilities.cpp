// Utilities.cpp: implementation of the sveral utility functions.
//////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
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

void print(string s, Eigen::MatrixXd matrix)
{
    std::cout << s << " " << matrix.rows() << " x " << matrix.cols() << std::endl;
    std::cout << matrix << std::endl;
}
