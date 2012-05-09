// Utilities.h: header with prototypes of utility functions.
//////////////////////////////////////////////////////////////////////

#if !defined(LECTURE_372FC_UTILITIES)
#define LECTURE_372FC_UTILITIES

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

#include <iostream>
#include <GL/gl.h>
#include <eigen3/Eigen/Core>
#include <string>

using namespace std;

///////////////////////////////////
// error/warning message display //
///////////////////////////////////
#define MESSAGE_STRING_SIZE 1000
static char message_string[MESSAGE_STRING_SIZE];

typedef enum {ERROR_MESSAGE, WARNING_MESSAGE} MESSAGE_TYPE;
void displayMessage(MESSAGE_TYPE type, char* message, ...);

////////////////////////////////////
//    2D array of 3D vertices     //
//	  ( stored as float[3] )      //
////////////////////////////////////

class CVertexArray2D
{
public:
	CVertexArray2D();
	CVertexArray2D(int m, int n); // (rows, columns)
	virtual ~CVertexArray2D();
	void setSize(int m, int n); // (rows, columns)
	int getN() const { return sizeN;}
	int getM() const { return sizeM;}
	bool isEmpty() { return (data==NULL);}
	float** operator[](int row);			// returns a column of vertices 
	float* operator()(int row, int column);	// returns a vertex (type float[3])
	friend ostream& operator<<(ostream&, CVertexArray2D&);
protected:
	float*** data;	// the actual array of data
	int sizeM;		// # of rows 
	int sizeN;		// # of columns 
};


/////////////////////////////////
//    basic math utilities     //
/////////////////////////////////

inline float fmax(float a, float b){ return (a>b)?a:b; }
inline float fmin(float a, float b){ return (a<b)?a:b; }
inline int imax(int a, int b){ return (a>b)?a:b; }
inline int imin(int a, int b){ return (a<b)?a:b; }
double sinc(double x);

///////////////////////////////////
//    input/output utilities     //
///////////////////////////////////

void print(string s, Eigen::VectorXd vector);
void print(string s, Eigen::Vector3d vector);
void print(string s, Eigen::MatrixXd matrix);
void clear(Eigen::VectorXd& vec);
void clear(Eigen::MatrixXd& mat);
void clearCross(Eigen::MatrixXd& mat, int row, int col);
void identity(Eigen::MatrixXd& mat);

// skips line when reading an input file
inline void skipLine(istream& s) { char c; 	do  s.get(c); while (c != '\n'); }


///////////////////////////////////
//      random numbers           //
///////////////////////////////////

void sgenrand(unsigned long seed);
unsigned long genrand();
double genrand2();

// integer random number in [min,max]
inline int randomInt(int min, int max)
{
	return min+genrand()%(max-min+1);
}

// double random number in [min,max]
inline double randomDouble(float min, double max)
{
	return min+(genrand2())*(max-min);
}

static const double PI = 3.1415926535897932384626433832795028841971693993751058;
#endif // !defined(LECTURE_372FC_UTILITIES)
