// Trackball.h: interface for the CTrackball class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TRACKBALL2_H__D9D7AFA2_744F_11D2_B4F5_0000E8D74310__INCLUDED_)
#define AFX_TRACKBALL2_H__D9D7AFA2_744F_11D2_B4F5_0000E8D74310__INCLUDED_


#include <GL/gl.h>

class CTrackball
{
public:
	CTrackball();
	virtual ~CTrackball();
	void tbInit(GLuint button);
	void tbMatrix();
	void tbReshape(int width, int height);
	void tbMouse(int button, int state, int x, int y);
	void tbKeyboard(int key);
	void tbMotion(int x, int y);
private:
	GLuint    tb_lasttime;
	GLfloat   tb_lastposition[3];
	
	GLfloat   tb_angle;
	GLfloat   tb_axis[3];
	GLfloat   tb_transform[4][4];

	GLuint    tb_width;
	GLuint    tb_height;

	GLint     tb_button;
	GLboolean tb_tracking;

	void _tbPointToVector(int x, int y, int width, int height, float v[3]);
	void _tbStartMotion(int x, int y, int button, int time);
	void _tbStopMotion(int button, unsigned time);
};

#endif // !defined(AFX_TRACKBALL2_H__D9D7AFA2_744F_11D2_B4F5_0000E8D74310__INCLUDED_)
