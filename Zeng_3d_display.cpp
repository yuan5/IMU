#include "Zeng_3d_display.h"

#include <QDebug>

#include <GL/glu.h>
#include <GL/gl.h>

#include <QOpenGLShaderProgram>

#include <math.h>

#include "zeng_data_receive.h"



#define Pi (3.141592653f)

Zeng_3d_display::Zeng_3d_display(QWidget *parent):QOpenGLWidget(parent)
{
//	setGeometry(0,0,640,480);
}


void Zeng_3d_display::initializeGL()
{
//    qDebug()<<"初始化 GL"<<endl;
    initializeOpenGLFunctions();

    glShadeModel(GL_SMOOTH);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    initialTexture();
}

float *euler_angle = dis_data.euler;
extern float rad_deg_sw;
Vector3f linar_acc;
void Zeng_3d_display::paintGL()
{
	Vector3f temp_acc;
//	float a = 0;
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	float c1 = cos(euler_angle[0]/(2*rad_deg_sw));
	float c2 = cos(euler_angle[1]/(2*rad_deg_sw));
	float c3 = cos(euler_angle[2]/(2*rad_deg_sw));

	float s1 = sin(euler_angle[0]/(2*rad_deg_sw));
	float s2 = sin(euler_angle[1]/(2*rad_deg_sw));
	float s3 = sin(euler_angle[2]/(2*rad_deg_sw));

    float x = (s1*c2*c3-c1*s2*s3)*180.0/3.141592;
    float y = (c1*s2*c3+s1*c2*s3)*180.0/3.141592;
    float z = (c1*c2*s3-s1*s2*c3)*180.0/3.141592;
    float w = acos(c1*c2*c3+s1*s2*s3)*2.0*180.0/3.141592;

	temp_acc << dis_data.acc[0], dis_data.acc[1], dis_data.acc[2];
//	a = sqrt(dis_data.acc[0]*dis_data.acc[0]+dis_data.acc[1]*dis_data.acc[1]+dis_data.acc[2]*dis_data.acc[2]);
	linar_acc = dis_data.Rotation_matrix*temp_acc - Vector3f(0,0,1.0f);
////	linar_acc += 0.3*(temp_acc-linar_acc);
//	if((a>1.05)||(a<0.95))
//	{
//		linar_acc += 0.1*temp_acc;
//	}else{
//		linar_acc *= 0.1f;
//	}

    glLoadIdentity();
    gluLookAt(-4.5, 0.3, 3, 0, 0, 0, 0, 0, 1);
	glTranslatef(0.0, 0.0, 0.0);
//	glTranslatef(linar_acc(0), linar_acc(1), linar_acc(2));
	//dis_data
    glRotatef(w, x, y, z);
    glBegin(GL_QUADS);
    {
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(1.0, 1.0, -0.40);
        glVertex3f(-1.0, 1.0, -0.40);
        glVertex3f(-1.0, 1.0, 0.40);
        glVertex3f(1.0, 1.0, 0.40);

        glColor3f(.50, 0.50, 0.50);
        glVertex3f(1.0, -1.0, .40);
        glVertex3f(-1.0, -1.0, .40);
        glVertex3f(-1.0, -1.0, -0.40);
        glVertex3f(1.0, -1.0, -0.40);

        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(1.0, 1.0, .40);
        glVertex3f(-1.0, 1.0, .40);
        glVertex3f(-1.0, -1.0, .40);
        glVertex3f(1.0, -1.0, .40);

        glColor3f(.30, .30, 0.30);
        glVertex3f(1.0, -1.0, -0.40);
        glVertex3f(-1.0, -1.0, -0.40);
        glVertex3f(-1.0, 1.0, -0.40);
        glVertex3f(1.0, 1.0, -0.40);

        glColor3f(0.20, 0.20, 0.20);
        glVertex3f(-1.0, 1.0, .40);
        glVertex3f(-1.0, 1.0, -0.40);
        glVertex3f(-1.0, -1.0, -0.40);
        glVertex3f(-1.0, -1.0, .40);

        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(1.0, 1.0, -0.40);
        glVertex3f(1.0, 1.0, .40);
        glVertex3f(1.0, -1.0, .40);
        glVertex3f(1.0, -1.0, -0.40);
    }
    glEnd();
    glBegin(GL_LINES);
    {
        glColor3f(1,0.5,0.5);
        glVertex3f(0,0,0);
        glVertex3f(2,0,0);

        glColor3f(0.50,0.5,0.50);
        glVertex3f(0,0,0);
		glVertex3f(0,2,0);

        glColor3f(0.5,0.5,1);
        glVertex3f(0,0,0);
        glVertex3f(0,0,2);
    }
    glEnd();

    if(HD_texture)
    {
        HD_texture->bind();
    }
    glColor3f(1.0,1.0,1.0);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    {
        glTexCoord2d(1.0, 1.0);
        glVertex3f(1.0, 1.0, .40);

        glTexCoord2d(0.0, 1.0);
        glVertex3f(-1.0, 1.0, .40);

        glTexCoord2d(0.0, 0.0);
        glVertex3f(-1.0, -1.0, .40);

        glTexCoord2d(1.0, 0.0);
        glVertex3f(1.0, -1.0, .40);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);


    glLoadIdentity();
    gluLookAt(-4.5, 0.3, 3, 0, 0, 0, 0, 0, 1);
//    glTranslatef(0, 0.0, 0.0);
    glBegin(GL_LINES);
    {
        glColor3f(1,0,0);
        glVertex3f(0,0,0);
		glVertex3f(2.5,0,0);

        glColor3f(0,1,0);
        glVertex3f(0,0,0);
		glVertex3f(0,2.5,0);

        glColor3f(0,0,1);
        glVertex3f(0,0,0);
        glVertex3f(0,0,2.5);
    }
    glEnd();

	glBegin(GL_LINES);
	{
		glColor3f(1,0,0);
		glVertex3f(0,0,0);
		glVertex3f(linar_acc(0),linar_acc(1),linar_acc(2));
	}
	glEnd();
}

void Zeng_3d_display::resizeGL(int w, int h)
{
    if(h == 0)
    {
        h = 1;
    }

    glViewport(0, 0, (GLint)w, (GLint)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

void Zeng_3d_display::initialTexture()
{
    texture_image = new QImage("./image/imu_q.png");
    HD_texture = new QOpenGLTexture(texture_image->mirrored());

    HD_texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
    HD_texture->setMagnificationFilter(QOpenGLTexture::Linear);
}


