// Geometry.h: interface for the Geometry class.
// Copyright: (c) Burkhard Wuensche, 1997
//////////////////////////////////////////////////////////////////////

#if !defined(LECTURE_372FC_GEOMETRY)
#define LECTURE_372FC_GEOMETRY

#include <iostream>
using namespace std;
#include <math.h>
#include "Utilities.h"

typedef enum { X, Y, Z, W } Axis;	// includes 3D homogenous coordinates

#define ZEROTOL 1.0e-30
const float Eps = 1.0e-5f;
const float Pi = 3.14159265358979323846264338327f;
const float Euler = 2.71828182845904523536028747135f;

//int pow(int a, int b);

// 3D double vector
class CVec3df {
	public:
	    // some good friends
	    friend class CMatrix3df;

	    // Constructors/ Destructor
	    CVec3df();
	    CVec3df(float x, float y, float z);
		CVec3df(const CVec3df& v); // Copy constructor
		virtual ~CVec3df();

	    // Assignment operator
	    CVec3df& operator=(const CVec3df& v1);

		// Vector in array form
		float* getArray() { return v;};
		void getArray(float *array) { array[0]=v[0]; array[1]=v[1]; array[2]=v[2];};
		void setArray(float *array) { v[0]=array[0]; v[1]=array[1]; v[2]=array[2];};
		void getFloatArray(float *array) { array[0]=(float) v[0]; array[1]=(float) v[1]; array[2]=(float) v[2];};

	    // some other operators
	    float& operator[](int index) const { return v[index]; }
	    CVec3df& operator+=(const CVec3df& v1);
	    CVec3df& operator-=(const CVec3df& v1);
	    CVec3df& operator*=(float scalar);
	    CVec3df& operator/=(float scalar);
		void setVector(float v0, float v1, float v2) {v[0]=v0; v[1]=v1; v[2]=v2;}
		void getVector(float& v0, float& v1, float& v2) {v0=v[0]; v1=v[1]; v2=v[2];}
		void setVector(int axis, float value) {v[axis]=value;}
		void getVector(int axis, float& value) {value=v[axis];}

	    friend CVec3df operator+(const CVec3df& v1, const CVec3df& v2);
	    friend CVec3df operator-(const CVec3df& v1, const CVec3df& v2);
	    friend CVec3df operator*(float scalar, const CVec3df& v1);
	    friend CVec3df operator*(const CVec3df& v1, float scalar);
	    friend CVec3df operator/(const CVec3df& v1, float scalar);
	    friend CVec3df operator*(const CVec3df& v1, const CVec3df& v2);
	    friend CVec3df operator-(const CVec3df& v1);
	    friend bool operator==(const CVec3df& v1, const CVec3df& v2);
	    friend bool operator!=(const CVec3df& v1, const CVec3df& v2);

	    // I/O operator
	    friend ostream& operator<<(ostream& s, const CVec3df& v1);
	    friend istream& operator>>(istream& s, CVec3df& v1);

	    // length
	    float length(void) const;

	    // normalize
	    CVec3df normalise(void) const;		// returns a normalised vector
		void normaliseDestructive(void);	// normalises the calling vector object
		void normaliseDestructiveNoError(void);	// normalises the calling vector object 
												// no error if vector has length zero
	    // dot product
	    float dot(const CVec3df& v1) const;

	    // cross product
	    CVec3df cross(const CVec3df& v1) const;

		// rotate angle degree around axis i (0 or X -> x-axis, 1 or Y -> y-axis, 2 or Z -> z-axis)
		CVec3df rotate(const int i, float angle) const; 

		// reflect v1 on the calling vector
		CVec3df reflect(const CVec3df& v1) const;

		// decomposes the calling vector into two components linear and orthogonal to n
		void decompose(const CVec3df& n, CVec3df& vLinear, CVec3df& vOrthogonal);
	private:
	    float* v;
};

// more convienent way to use the dot and cross products
inline float dot(const CVec3df& v1, const CVec3df& v2) { return v1.dot(v2); } 
inline CVec3df cross(const CVec3df& v1, const CVec3df& v2) { return v1.cross(v2); } 
inline CVec3df fabs(const CVec3df& v) { return CVec3df((float) fabs(v[X]), (float) fabs(v[Y]), (float) fabs(v[Z])); }
inline float length(const CVec3df& v) { return v.length();}
inline CVec3df reflect(const CVec3df& v, const CVec3df& n) { return n.reflect(v);}

// Some useful vectors (zero vector and 3D cartesian basis vectors)
const CVec3df ZeroVector3df = CVec3df(0.0, 0.0, 0.0);
const CVec3df E1_3df = CVec3df(1.0, 0.0, 0.0);
const CVec3df E2_3df = CVec3df(0.0, 1.0, 0.0);
const CVec3df E3_3df = CVec3df(0.0, 0.0, 1.0);


// 3D float Matrix
class CMatrix3df {
	public:
	    // Constructors/ Destructor
	    CMatrix3df();
	    CMatrix3df(float a11, float a12, float a13,
		   float a21, float a22, float a23,
		   float a31, float a32, float a33);
	    CMatrix3df(const CVec3df& v1,const CVec3df& v2,const CVec3df& v3);
	    CMatrix3df(const CMatrix3df& m1); // Copy constructor
		virtual ~CMatrix3df();

	    // Assignment operator
	    CMatrix3df& operator=(const CMatrix3df& m1);

	    // some other operators
	    CVec3df operator[](const int i) const { return CVec3df(m[i],m[3+i],m[6+i]); }
	    float& operator()(int i, int j) const { return m[3*i+j]; }
	    CMatrix3df& operator+=(const CMatrix3df& m1);
	    CMatrix3df& operator-=(const CMatrix3df& m1);
	    CMatrix3df& operator*=(const CMatrix3df& m1);
	    CMatrix3df& operator*=(float scalar);
	    CMatrix3df& operator/=(float scalar);
	    friend CMatrix3df operator*(float scalar, const CMatrix3df& m1);
	    friend CMatrix3df operator*(const CMatrix3df& m1, float scalar);
	    friend CMatrix3df operator/(const CMatrix3df& m1, float scalar);
	    friend CVec3df operator*(const CMatrix3df& m1, const CVec3df& v1);
	    friend CVec3df operator*(const CVec3df& v1, const CMatrix3df& m1);
	    friend CMatrix3df operator+(const CMatrix3df& m1, const CMatrix3df& m2);
	    friend CMatrix3df operator*(const CMatrix3df& m1, const CMatrix3df& m2);

	    // I/O operator
	    friend ostream& operator<<(ostream& s, const CMatrix3df& m1);
	    friend istream& operator>>(istream& s, CMatrix3df& m1);

	    // calculation of the transposed CMatrix
	    CMatrix3df transpose(void) const;

	    // calculation of the inverse CMatrix
	    CMatrix3df inverse(void) const;

	    // calculation of the determinante
	    float det(void) const;

	private:
		float* m;	// matrix stored in a 1D array row wise
};

// more convienent way to use the transpose method
inline CMatrix3df transpose(CMatrix3df& m1) { return m1.transpose(); }
// more convienent way to obtain a normalised vector
inline CVec3df normalise(CVec3df& v) { return v.normalise(); }

// some useful matrices (zero matrix and identityMatrix
const CMatrix3df M0_3df = CMatrix3df(ZeroVector3df, ZeroVector3df, ZeroVector3df);
const CMatrix3df M1_3df = CMatrix3df(E1_3df, E2_3df, E3_3df);

class CMatrix2df;

// 2D float vector
class CVec2df {
	public:
	    // Constructors/ Destructor
	    CVec2df();
	    CVec2df(float x, float y);
	    CVec2df(const CVec2df& v); // Copy constructor
		virtual ~CVec2df();

	    // Assignment operator
	    CVec2df& operator=(const CVec2df& v1);

		// Vector in array form
		float* getArray() { return v;};
		void getArray(float *array) { array[0]=v[0]; array[1]=v[1];};
		void setArray(float *array) { v[0]=array[0]; v[1]=array[1];};
		void getFloatArray(float *array) { array[0]=(float) v[0]; array[1]=(float) v[1];};

		// some other operators
	    float& operator[](int index) const { return v[index]; }
	    CVec2df& operator+=(const CVec2df& v1);
	    CVec2df& operator-=(const CVec2df& v1);
	    CVec2df& operator*=(float scalar);
	    CVec2df& operator/=(float scalar);
		void setVector(float v0, float v1) {v[0]=v0; v[1]=v1;}
		void getVector(float& v0, float& v1) {v0=v[0]; v1=v[1];}
	    friend CVec2df operator+(const CVec2df& v1, const CVec2df& v2);
	    friend CVec2df operator-(const CVec2df& v1, const CVec2df& v2);
	    friend bool operator==(const CVec2df& v1, const CVec2df& v2);
	    friend CVec2df operator*(float scalar, const CVec2df& v1);

	    // I/O operator
	    friend ostream& operator<<(ostream& s, const CVec2df& v1);
	    friend istream& operator>>(istream& s, CVec2df& v1);

	    // length
	    float length(void) const;

	    // normalize
	    CVec2df normalise(void) const;
		void normaliseDestructive(void);
		void normaliseDestructiveNoError(void);

	    // dot product
	    float dot(const CVec2df& v1) const;

	    // some good friends
	    friend class CMatrix2df;
	    friend CMatrix2df operator*(const CMatrix2df& m1, const CMatrix2df& m2);

	private:
	    float* v;
};

// more convienent way to use the dot and cross products
inline float dot(const CVec2df& v1, const CVec2df& v2) { return v1.dot(v2); } 

// Some useful CVectorsconst CVec3df ZeroVector3df = CVec3df(0.0, 0.0, 0.0);
const CVec2df ZeroVector2df = CVec2df(0.0, 0.0);
const CVec2df E1_2df = CVec2df(1.0, 0.0);
const CVec2df E2_2df = CVec2df(0.0, 1.0);


// 2D float Matrix
class CMatrix2df {
	public:
	    // Constructors/ Destructor
	    CMatrix2df();
	    CMatrix2df(float a11, float a12, float a21, float a22);
	    CMatrix2df(const CVec2df& v1, const CVec2df& v2);
	    CMatrix2df(const CMatrix2df& m1); // Copy constructor
		virtual ~CMatrix2df();

	    // Assignment operator
	    CMatrix2df& operator=(const CMatrix2df& m1);

	    // some other operators
	    CVec2df operator[](const int i) const { return CVec2df(m[i],m[2+i]); }
	    float& operator()(int i, int j) const { return m[2*i+j]; }
	    friend CMatrix2df operator*(float scalar, const CMatrix2df& m1);
	    friend CVec2df operator*(const CMatrix2df& m1, const CVec2df& v1);
	    friend CMatrix2df operator+(const CMatrix2df& m1, const CMatrix2df& m2);
	    friend CMatrix2df operator*(const CMatrix2df& m1, const CMatrix2df& m2);

	    // I/O operator
	    friend ostream& operator<<(ostream& s, const CMatrix2df& m1);
	    friend istream& operator>>(istream& s, CMatrix2df& m1);

	    // calculation of the transposed CMatrix
	    CMatrix2df transpose(void) const;

	    // calculation of the inverse CMatrix
	    CMatrix2df inverse(void) const;

	    // calculation of the determinante
	    float det(void) const;

	private:
		float* m;
};

// more convienent way to use the transpose method
inline CMatrix2df transpose(CMatrix2df& m1) { return m1.transpose(); }

// some useful matrices
const CMatrix2df M0_2df = CMatrix2df(ZeroVector2df, ZeroVector2df);
const CMatrix2df M1_2df = CMatrix2df(E1_2df, E2_2df);



// 3D float undirected edge
class CEdge3df {
	public:
	    // Constructors/ Destructor
	    CEdge3df();
	    CEdge3df(const CVec3df& v1, const CVec3df& v2);
	    CEdge3df(const CEdge3df& e); // Copy constructor
		virtual ~CEdge3df(){};

	    // Assignment operator
	    CEdge3df& operator=(const CEdge3df& v1);

	    // some other operators
		CVec3df getDirection() const {return v2-v1; }
		CVec3df getV1() const {return v1; }
		CVec3df getV2() const {return v2; }
		friend bool operator==(const CEdge3df e1, const CEdge3df e2);

	    // I/O operator
	    friend ostream& operator<<(ostream& s, const CEdge3df& v1);
//	    friend istream& operator>>(istream& s, CEdge3df& v1);

	private:
	    CVec3df v1,v2;
};

class CPlane3df
{
public:
	CPlane3df():a(0),b(0),c(1),d(0){}
	CPlane3df(float a, float b, float c, float d){ this->a=a; this->b=b; this->c=c; this->d=d;}
	CPlane3df(const CVec3df& p1, const CVec3df& p2, const CVec3df& p3);
	virtual ~CPlane3df(){}
	CVec3df getNormal() const{ return CVec3df(a,b,c);}
	float distance(const CVec3df& p) const{ return (a*p[0]+b*p[1]+c*p[2]-d)/(float) sqrt(a*a+b*b+c*c);}
	bool isAbove(const CVec3df& p) const{ return (distance(p)>0);}
    friend ostream& operator<<(ostream& s, const CPlane3df& p);
private:
	float a,b,c,d;
};

#endif // !defined(LECTURE_372FC_GEOMETRY)
