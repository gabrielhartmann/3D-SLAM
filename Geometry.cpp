// Geometry.cpp: implementation of the Geometry class.
// Copyright: (c) Burkhard Wuensche, 1997
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdlib.h>
#include "Geometry.h"
#include "Utilities.h"

//////////////////////////////////////////////////////////////////////
// CVec3df Class
//////////////////////////////////////////////////////////////////////

CVec3df::CVec3df()
{
	v=new float[3];
	v[X] = 0.0; v[Y] = 0.0; v[Z] = 0.0;
}

CVec3df::CVec3df(float x, float y, float z)
{
	v=new float[3];
	v[X] = x; v[Y] = y; v[Z] = z;
}

CVec3df::CVec3df(const CVec3df& v1)
{
	v=new float[3];
	v[X] = v1.v[X]; v[Y] = v1.v[Y]; v[Z] = v1.v[Z];
}

CVec3df::~CVec3df()
{
	delete [] v;
}

CVec3df& CVec3df::operator=(const CVec3df& v1)
{
	v[X] = v1.v[X]; v[Y] = v1.v[Y]; v[Z] = v1.v[Z];
	return *this;
}

CVec3df& CVec3df::operator+=(const CVec3df& v1)
{
	v[X] += v1.v[X]; v[Y] += v1.v[Y]; v[Z] += v1.v[Z];
	return *this;
}

CVec3df& CVec3df::operator-=(const CVec3df& v1)
{
	v[X] -= v1.v[X]; v[Y] -= v1.v[Y]; v[Z] -= v1.v[Z];
	return *this;
}

CVec3df& CVec3df::operator*=(float scalar)
{
	v[X] *= scalar; v[Y] *= scalar; v[Z] *= scalar;
	return *this;
}

CVec3df& CVec3df::operator/=(float scalar)
{
	v[X] /= scalar; v[Y] /= scalar; v[Z] /= scalar;
	return *this;
}

CVec3df operator+(const CVec3df& v1, const CVec3df& v2)
{
	return CVec3df(v1.v[X] + v2.v[X], v1.v[Y] + v2.v[Y], v1.v[Z] + v2.v[Z]);
}

CVec3df operator-(const CVec3df& v1, const CVec3df& v2)
{
	return CVec3df(v1.v[X] - v2.v[X], v1.v[Y] - v2.v[Y], v1.v[Z] - v2.v[Z]);
}

CVec3df operator*(float scalar, const CVec3df& v1)
{
	CVec3df v0 = v1;
	v0 *= scalar;
	return v0;
}

CVec3df operator*(const CVec3df& v1, float scalar)
{
	CVec3df v0 = v1;
	v0 *= scalar;
	return v0;
}

CVec3df operator/(const CVec3df& v1, float scalar)
{
	CVec3df v0 = v1;
	v0 /= scalar;
	return v0;
}

CVec3df operator*(const CVec3df& v1, const CVec3df& v2)
{
	CVec3df v0 = v1;
	v0.v[X] *= v2.v[X]; v0.v[Y] *= v2.v[Y]; v0.v[Z] *= v2.v[Z];
	return v0;
}

CVec3df operator-(const CVec3df& v1)
{
	return CVec3df(-v1.v[X], -v1.v[Y], -v1.v[Z]);
}

bool operator==(const CVec3df& v1, const CVec3df& v2)
{
	return (v1.v[X]==v2.v[X] && v1.v[Y]==v2.v[Y] && v1.v[Z]==v2.v[Z]); 
}

bool operator!=(const CVec3df& v1, const CVec3df& v2)
{
	return (v1.v[X]!=v2.v[X] || v1.v[Y]!=v2.v[Y] || v1.v[Z]!=v2.v[Z]); 
}

ostream& operator<<(ostream& s, const CVec3df& v1)
{
	return s << '(' << v1.v[X] << ", " << v1.v[Y] << ", " << v1.v[Z] << ')';
}

istream& operator>>(istream& s, CVec3df& v1)
{
	char c = 0;

	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CVec3df::>>. bad input format");
	s >> v1.v[X];
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CVec3df::>>. bad input format");
	s >> v1.v[Y];
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CVec3df::>>. bad input format");
	s >> v1.v[Z];
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CVec3df::>>. bad input format");

	return s;
}

float CVec3df::length(void) const
{
	return (float) sqrt(this->dot(*this));
}

CVec3df CVec3df::normalise(void) const
{
	CVec3df v0 = (*this);
	float length=v0.length();
	if (length != 0)
	    v0 /= length;
	else
		displayMessage(ERROR_MESSAGE, "CVec3df::normalise. zero length");
	return v0;
}

void CVec3df::normaliseDestructive(void)
{
	float length=this->length();
	if (length != 0)
	    *this /= length;
	else
		displayMessage(ERROR_MESSAGE, "CVec3df::normaliseDestructive. zero length");
}

void CVec3df::normaliseDestructiveNoError(void)
{
	float length=this->length();
	if (length != 0)
	    *this /= length;
	else
		setVector(0,0,0);	// return zero vector if normal has length zero!!
}


float CVec3df::dot(const CVec3df& v1) const
{
	float dot = v[X] * v1.v[X] + v[Y] * v1.v[Y] + v[Z] * v1.v[Z];
	return dot;
}

CVec3df CVec3df::cross(const CVec3df& v1) const
{
	CVec3df v0(v[Y] * v1.v[Z] - v[Z] * v1.v[Y],
		  v[Z] * v1.v[X] - v[X] * v1.v[Z],
		  v[X] * v1.v[Y] - v[Y] * v1.v[X]);
	return v0;
}

CVec3df CVec3df::rotate(const int i, float angle) const
{
	CMatrix3df m;
	float a,b;
	angle=angle/180*Pi;	// convert radian to degree
	switch (i)
	{
	case 0: m(0,0)=1.0; a=(float) cos(angle); b=(float) sin(angle);
			m(1,1)=a; m(2,2)=a; m(2,1)=-b; m(1,2)=b; break;
	case 1: m(1,1)=1.0; a=(float) cos(angle); b=(float) sin(angle);
			m(0,0)=a; m(2,2)=a; m(2,0)=-b; m(0,2)=b; break;
	case 2: m(2,2)=1.0; a=(float) cos(angle); b=(float) sin(angle);
			m(0,0)=a; m(1,1)=a; m(0,1)=-b; m(1,0)=b; break;
	}
	return m*(*this);
}

CVec3df CVec3df::reflect(const CVec3df& v1) const
{
	CVec3df n=this->normalise();
	return v1-2*n.dot(v1)*n;
}

void CVec3df::decompose(const CVec3df& nn, CVec3df& vLinear, CVec3df& vOrthogonal)
{
	CVec3df n=nn.normalise();
	vLinear=this->dot(n)*n;
	vOrthogonal=*this-vLinear;
}

//////////////////////////////////////////////////////////////////////
// CMatrix3df Class
//////////////////////////////////////////////////////////////////////

CMatrix3df::CMatrix3df()
{
	m=new float[9];
	for(int i=0;i<9;i++) m[i]=0.0;
}

CMatrix3df::CMatrix3df(float a11, float a12, float a13,
	       float a21, float a22, float a23,
	       float a31, float a32, float a33)
{
	m=new float[9];
	m[0]=a11; m[1]=a12; m[2]=a13;
	m[3]=a21; m[4]=a22; m[5]=a23;
	m[6]=a31; m[7]=a32; m[8]=a33;
}

CMatrix3df::CMatrix3df(const CVec3df& v1, const CVec3df& v2, const CVec3df& v3)
{
	m=new float[9];
	m[0]=v1.v[0]; m[1]=v2.v[0]; m[2]=v3.v[0];
	m[3]=v1.v[1]; m[4]=v2.v[1]; m[5]=v3.v[1];
	m[6]=v1.v[2]; m[7]=v2.v[2]; m[8]=v3.v[2];
}

CMatrix3df::CMatrix3df(const CMatrix3df& m1)
{
	m=new float[9];
	for(int i=0;i<9; i++) m[i]=m1.m[i];
}

CMatrix3df::~CMatrix3df()
{
	delete [] m;
}

CMatrix3df& CMatrix3df::operator=(const CMatrix3df& m1)
{
	for(int i=0;i<9; i++) m[i]=m1.m[i];
	return *this;
}

CMatrix3df& CMatrix3df::operator+=(const CMatrix3df& m1)
{
	for(int i=0;i<9; i++) m[i]+=m1.m[i];
	return *this;
}

CMatrix3df& CMatrix3df::operator-=(const CMatrix3df& m1)
{
	for(int i=0;i<9; i++) m[i]-=m1.m[i];
	return *this;
}

CMatrix3df& CMatrix3df::operator*=(const CMatrix3df& m1)
{
	m[0]=m[0]*m1.m[0]+m[1]*m1.m[3]+m[2]*m1.m[6];
	m[1]=m[0]*m1.m[1]+m[1]*m1.m[4]+m[2]*m1.m[7];
	m[2]=m[0]*m1.m[2]+m[1]*m1.m[5]+m[2]*m1.m[8];
	m[3]=m[3]*m1.m[0]+m[4]*m1.m[3]+m[5]*m1.m[6];
	m[4]=m[3]*m1.m[1]+m[4]*m1.m[4]+m[5]*m1.m[7];
	m[5]=m[3]*m1.m[2]+m[4]*m1.m[5]+m[5]*m1.m[8];
	m[6]=m[6]*m1.m[0]+m[7]*m1.m[3]+m[8]*m1.m[6];
	m[7]=m[6]*m1.m[1]+m[7]*m1.m[4]+m[8]*m1.m[7];
	m[8]=m[6]*m1.m[2]+m[7]*m1.m[5]+m[8]*m1.m[8];
	return *this;
}

CMatrix3df& CMatrix3df::operator*=(float scalar)
{
	for(int i=0;i<9; i++) m[i]*=scalar;
	return *this;
}

CMatrix3df& CMatrix3df::operator/=(float scalar)
{
	for(int i=0;i<9; i++) m[i]/=scalar;
	return *this;
}

CMatrix3df operator*(float scalar, const CMatrix3df& m1)
{
	CMatrix3df m0 = CMatrix3df(m1.m[0]*scalar, m1.m[1]*scalar, m1.m[2]*scalar,
						m1.m[3]*scalar, m1.m[4]*scalar, m1.m[5]*scalar,
						m1.m[6]*scalar, m1.m[7]*scalar, m1.m[8]*scalar);
	return m0;
}

CMatrix3df operator*(const CMatrix3df& m1, float scalar)
{
	CMatrix3df m0 = CMatrix3df(m1.m[0]*scalar, m1.m[1]*scalar, m1.m[2]*scalar,
						m1.m[3]*scalar, m1.m[4]*scalar, m1.m[5]*scalar,
						m1.m[6]*scalar, m1.m[7]*scalar, m1.m[8]*scalar);
	return m0;
}

CMatrix3df operator/(const CMatrix3df& m1, float scalar)
{
	CMatrix3df m0 = CMatrix3df(m1.m[0]/scalar, m1.m[1]/scalar, m1.m[2]/scalar,
						m1.m[3]/scalar, m1.m[4]/scalar, m1.m[5]/scalar,
						m1.m[6]/scalar, m1.m[7]/scalar, m1.m[8]/scalar);
	return m0;
}

CVec3df operator*(const CMatrix3df& m1, const CVec3df& v1)
{
	CVec3df v0 = CVec3df(m1.m[0]*v1[0]+m1.m[1]*v1[1]+m1.m[2]*v1[2],
						m1.m[3]*v1[0]+m1.m[4]*v1[1]+m1.m[5]*v1[2],
						m1.m[6]*v1[0]+m1.m[7]*v1[1]+m1.m[8]*v1[2]);
	return v0;		
}

CVec3df operator*(const CVec3df& v1, const CMatrix3df& m1)
{
	CVec3df v0 = CVec3df(v1[0]*m1.m[0]+v1[1]*m1.m[3]+v1[2]*m1.m[6],
						v1[0]*m1.m[1]+v1[1]*m1.m[4]+v1[2]*m1.m[7],
						v1[0]*m1.m[2]+v1[1]*m1.m[5]+v1[2]*m1.m[8]);
	return v0;		
}

CMatrix3df operator+(const CMatrix3df& m1, const CMatrix3df& m2)
{
	CMatrix3df m0 = CMatrix3df(m1.m[0]+m2.m[0], m1.m[1]+m2.m[1], m1.m[2]+m2.m[2],
						m1.m[3]+m2.m[3], m1.m[4]+m2.m[4], m1.m[5]+m2.m[5],
						m1.m[6]+m2.m[6], m1.m[7]+m2.m[7], m1.m[8]+m2.m[8]);
	return m0;
}

CMatrix3df operator*(const CMatrix3df& m1, const CMatrix3df& m2)
{
	CMatrix3df m0 = CMatrix3df(m1.m[0]*m2.m[0]+m1.m[1]*m2.m[3]+m1.m[2]*m2.m[6],
				m1.m[0]*m2.m[1]+m1.m[1]*m2.m[4]+m1.m[2]*m2.m[7],
				m1.m[0]*m2.m[2]+m1.m[1]*m2.m[5]+m1.m[2]*m2.m[8],
				m1.m[3]*m2.m[0]+m1.m[4]*m2.m[3]+m1.m[5]*m2.m[6],
				m1.m[3]*m2.m[1]+m1.m[4]*m2.m[4]+m1.m[5]*m2.m[7],
				m1.m[3]*m2.m[2]+m1.m[4]*m2.m[5]+m1.m[5]*m2.m[8],
				m1.m[6]*m2.m[0]+m1.m[7]*m2.m[3]+m1.m[8]*m2.m[6],
				m1.m[6]*m2.m[1]+m1.m[7]*m2.m[4]+m1.m[8]*m2.m[7],
				m1.m[6]*m2.m[2]+m1.m[7]*m2.m[5]+m1.m[8]*m2.m[8]);
	return m0;		
}

CMatrix3df CMatrix3df::transpose(void) const
{
	CMatrix3df m0 = CMatrix3df(m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]);
	return m0;
}

CMatrix3df CMatrix3df::inverse(void) const
{
	CMatrix3df m1;
	float determinant=det();
	m1.m[0]=CMatrix2df(m[4],m[5],m[7],m[8]).det()/determinant;
	m1.m[3]=-CMatrix2df(m[3],m[5],m[6],m[8]).det()/determinant;
	m1.m[6]=CMatrix2df(m[3],m[4],m[6],m[7]).det()/determinant;
	m1.m[1]=-CMatrix2df(m[1],m[2],m[7],m[8]).det()/determinant;
	m1.m[4]=CMatrix2df(m[0],m[2],m[6],m[8]).det()/determinant;
	m1.m[7]=-CMatrix2df(m[0],m[1],m[6],m[7]).det()/determinant;
	m1.m[2]=CMatrix2df(m[1],m[2],m[4],m[5]).det()/determinant;
	m1.m[5]=-CMatrix2df(m[0],m[2],m[3],m[5]).det()/determinant;
	m1.m[8]=CMatrix2df(m[0],m[1],m[3],m[4]).det()/determinant;
	return m1;
}

float CMatrix3df::det(void) const
{
	return m[0]*m[4]*m[8]+m[1]*m[5]*m[6]+m[2]*m[3]*m[7]-
			m[0]*m[5]*m[7]-m[1]*m[3]*m[8]-m[2]*m[4]*m[6];
}

ostream& operator<<(ostream& s, const CMatrix3df& m1)
{
	s << "((" << m1.m[0] << ',' << m1.m[1] << ',' << m1.m[2] << ')';
	s << ",(" << m1.m[3] << ',' << m1.m[4] << ',' << m1.m[5] << ')';
	s << ",(" << m1.m[6] << ',' << m1.m[7] << ',' << m1.m[8] << "))";
	return s;
}

istream& operator>>(istream& s, CMatrix3df& m1)
{
	char c = 0;

	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CMatrix3df::>>. bad input format");
	for(int i=0;i<3;i++)
	{
		s >> c;
		if (c != '(')
			displayMessage(ERROR_MESSAGE, "CMatrix3df::>>. bad input format");
		for(int j=0;j<3;j++)
		{
			s >> m1.m[3*i+j];
			if (j!=2)
			{
				s >> c;
				if (c != ',')
					displayMessage(ERROR_MESSAGE, "CMatrix3df::>>. bad input format");
			}
		}
		s >> c;
		if (c != ')')
			displayMessage(ERROR_MESSAGE, "CVec3df::>>. bad input format");
		if (i!=2)
		{
			s >> c;
			if (c != ',')
				displayMessage(ERROR_MESSAGE, "CMatrix3df::>>. bad input format");
		}
	}
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CMatrix3df::>>. bad input format");
	return s;
}




//////////////////////////////////////////////////////////////////////
// CVec2df Class
//////////////////////////////////////////////////////////////////////

CVec2df::CVec2df()
{
	v=new float[2];
	v[X] = 0.0; v[Y] = 0.0;
}

CVec2df::CVec2df(float x, float y)
{
	v=new float[2];
	v[X] = x; v[Y] = y;
}

CVec2df::CVec2df(const CVec2df& v1)
{
	v=new float[2];
	v[X] = v1.v[X]; v[Y] = v1.v[Y];
}

CVec2df::~CVec2df()
{
	delete [] v;
}

CVec2df& CVec2df::operator=(const CVec2df& v1)
{
	v[X] = v1.v[X]; v[Y] = v1.v[Y];
	return *this;
}

CVec2df& CVec2df::operator+=(const CVec2df& v1)
{
	v[X] += v1.v[X]; v[Y] += v1.v[Y];
	return *this;
}

CVec2df& CVec2df::operator-=(const CVec2df& v1)
{
	v[X] -= v1.v[X]; v[Y] -= v1.v[Y];
	return *this;
}

CVec2df& CVec2df::operator*=(float scalar)
{
	v[X] *= scalar; v[Y] *= scalar;
	return *this;
}

CVec2df& CVec2df::operator/=(float scalar)
{
	v[X] /= scalar; v[Y] /= scalar;
	return *this;
}

CVec2df operator+(const CVec2df& v1, const CVec2df& v2)
{
	return CVec2df(v1.v[X] + v2.v[X], v1.v[Y] + v2.v[Y]);
}

CVec2df operator-(const CVec2df& v1, const CVec2df& v2)
{
	return CVec2df(v1.v[X] - v2.v[X], v1.v[Y] - v2.v[Y]);
}

bool operator==(const CVec2df& v1, const CVec2df& v2)
{
	return (v1.v[X]==v2.v[X] && v1.v[Y]==v2.v[Y]); 
}


CVec2df operator*(float scalar, const CVec2df& v1)
{
	CVec2df v0 = v1;
	v0 *= scalar;
	return v0;
}

ostream& operator<<(ostream& s, const CVec2df& v1)
{
	return s << '(' << v1.v[X] << ", " << v1.v[Y] << ")";
}

istream& operator>>(istream& s, CVec2df& v1)
{
	char c = 0;

	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CVec2df::>>. bad input format");
	s >> v1.v[X];
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CVec2df::>>. bad input format");
	s >> v1.v[Y];
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CVec2df::>>. bad input format");

	return s;
}

float CVec2df::length(void) const
{
	return (float) sqrt(this->dot(*this));
}

CVec2df CVec2df::normalise(void) const
{
	CVec2df v0 = (*this);
	if (v0.length() != 0)
	    v0 /= v0.length();
	else
		displayMessage(ERROR_MESSAGE, "CVec2df::normalise. zero length");
	return v0;
}

void CVec2df::normaliseDestructive(void)
{
	float length=this->length();
	if (length != 0)
	    *this /= length;
	else
		displayMessage(ERROR_MESSAGE, "CVec2df::normaliseDestructive. zero length");
}

void CVec2df::normaliseDestructiveNoError(void)
{
	float length=this->length();
	if (length != 0) *this /= length;
}


float CVec2df::dot(const CVec2df& v1) const
{
	float dot = v[X] * v1.v[X] + v[Y] * v1.v[Y];
	return dot;
}


//////////////////////////////////////////////////////////////////////
// CMatrix2df Class
//////////////////////////////////////////////////////////////////////

CMatrix2df::CMatrix2df()
{
	m=new float[4];
	m[0]=0.0; m[1]=0.0; m[2]=0.0; m[3]=0.0;
}

CMatrix2df::CMatrix2df(float a11, float a12, float a21, float a22)
{
	m=new float[4];
	m[0]=a11; m[1]=a12; m[2]=a21; m[3]=a22;
}

CMatrix2df::CMatrix2df(const CVec2df& v1, const CVec2df& v2)
{
	m=new float[4];
	m[0]=v1.v[0]; m[1]=v2.v[0]; m[2]=v1.v[1]; m[3]=v2.v[1];
}

CMatrix2df::CMatrix2df(const CMatrix2df& m1)
{
	m=new float[4];
	m[0]=m1.m[0]; m[1]=m1.m[1]; m[2]=m1.m[2]; m[3]=m1.m[3];
}


CMatrix2df::~CMatrix2df()
{
	delete [] m;
}

CMatrix2df& CMatrix2df::operator=(const CMatrix2df& m1)
{
	m[0]=m1.m[0]; m[1]=m1.m[1]; m[2]=m1.m[2]; m[3]=m1.m[3]; 
	return *this;
}

CMatrix2df operator*(float scalar, const CMatrix2df& m1)
{
	CMatrix2df m0 = CMatrix2df(m1.m[0]*scalar, m1.m[1]*scalar, 
								m1.m[2]*scalar, m1.m[3]*scalar);
	return m0;
}

CVec2df operator*(const CMatrix2df& m1, const CVec2df& v1)
{
	CVec2df v0 = CVec2df(m1.m[0]*v1[0]+m1.m[1]*v1[1],
						m1.m[2]*v1[0]+m1.m[3]*v1[1]);
	return v0;		
}

CMatrix2df operator+(const CMatrix2df& m1, const CMatrix2df& m2)
{
	CMatrix2df m0 = CMatrix2df(m1.m[0]+m2.m[0], m1.m[1]+m2.m[1],
								m1.m[2]+m2.m[2], m1.m[3]+m2.m[3]);
	return m0;
}

CMatrix2df operator*(const CMatrix2df& m1, const CMatrix2df& m2)
{
	CMatrix2df m0 = CMatrix2df(m1.m[0]*m2.m[0]+m1.m[1]*m2.m[2],
								m1.m[0]*m2.m[1]+m1.m[1]*m2.m[3],
								m1.m[2]*m2.m[0]+m1.m[3]*m2.m[2],
								m1.m[2]*m2.m[1]+m1.m[3]*m2.m[3]);
	return m0;		
}

CMatrix2df CMatrix2df::transpose(void) const
{
	CMatrix2df m0 = CMatrix2df(m[0],m[2],m[1],m[3]);
	return m0;
}

CMatrix2df CMatrix2df::inverse(void) const
{
	CMatrix2df m1(m[3],-m[1],-m[2],m[0]);
	return (1.0f/det())*m1;
}

float CMatrix2df::det(void) const
{
	return m[0]*m[3]-m[1]*m[2];
}

ostream& operator<<(ostream& s, const CMatrix2df& m1)
{
	s << "((" << m1.m[0] << ',' << m1.m[1] << ')';
	s << ",(" << m1.m[2] << ',' << m1.m[3] << "))";
	return s;
}

istream& operator>>(istream& s, CMatrix2df& m1)
{
	char c = 0;

	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> m1.m[0];
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> m1.m[1];
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> c;
	if (c != '(')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> m1.m[2];
	s >> c;
	if (c != ',')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> m1.m[3];
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	s >> c;
	if (c != ')')
		displayMessage(ERROR_MESSAGE, "CMatrix2df::>>. bad input format");
	return s;
}



//////////////////////////////////////////////////////////////////////
// CEdge3df Class
//////////////////////////////////////////////////////////////////////

CEdge3df::CEdge3df()
{
	v1=CVec3df();
	v2=CVec3df();
}

CEdge3df::CEdge3df(const CVec3df& p1, const CVec3df& p2)
{
	v1=p1;
	v2=p2;
}

CEdge3df::CEdge3df(const CEdge3df& e)
{
	v1=e.v1;
	v2=e.v2;
}

CEdge3df& CEdge3df::operator=(const CEdge3df& e)
{
	v1=e.v1;
	v2=e.v2;
	return *this;
}

bool operator==(const CEdge3df e1, const CEdge3df e2)
{
	return ( ((e1.v1==e2.v1) && (e1.v2==e2.v2)) || ((e1.v1==e2.v2) && (e1.v2==e2.v1)) ); 
}

ostream& operator<<(ostream& s, const CEdge3df& edge)
{
	return s << '<' << edge.v1 << ", " << edge.v2 << '>';
}


//////////////////////////////////////////////////////////////////////
// CPlane3df Class
//////////////////////////////////////////////////////////////////////

CPlane3df::CPlane3df(const CVec3df& v1, const CVec3df& v2, const CVec3df& v3)
{
	CVec3df n=cross(v2-v1,v3-v1)+cross(v3-v2,v1-v2)+cross(v1-v3,v2-v3);
	n.normaliseDestructive();
	a=n[X]; b=n[Y]; c=n[Z];
	d=dot(v1,n);	
}

ostream& operator<<(ostream& s, const CPlane3df& p)
{
	return s << "Normal: (" << p.a << ", " << p.b << ", " << p.c << ") d=" << p.d;
}
