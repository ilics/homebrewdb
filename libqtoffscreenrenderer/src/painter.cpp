//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include <iostream>
#include "painter.h"


using namespace std;
using namespace cv;
using namespace Eigen;


float Painter::z_near;
float Painter::z_far;
int Painter::width;
int Painter::height;

SingletonPainter::SingletonPainter(float p_near,float p_far,int width,int height):
    QGLWidget(QGLFormat(QGLFormat::defaultFormat()),0),m_near(p_near),m_far(p_far)
{
    //create the framebuffer object - make sure to have a current context before creating it
    makeCurrent();
    m_fbo = new QOpenGLFramebufferObject(width,height,QOpenGLFramebufferObject::Depth);
    initializeOpenGLFunctions();
    m_background.setZero();
    m_objects.clear();
    m_color = Mat(height,width,CV_8UC3);
    m_depth = Mat(height,width, CV_32FC1);
    copy_rect.x=0;
    copy_rect.y=0;
    doneCurrent();

}

SingletonPainter::~SingletonPainter()
{
    delete m_fbo;
}


void SingletonPainter::paint(int x,int y,int w,int h)
{
    if(x==0&&w==0&&y==0&&h==0) render_rect = Rect(0,0,getWidth(),getHeight());
    else render_rect = Rect(x,y,w,h);
    copy_rect.width = render_rect.width;
    copy_rect.height = render_rect.height;
    paintGL();
}


void SingletonPainter::paintGL()
{
    makeCurrent();
    m_fbo->bind();

    glViewport(0,0,m_fbo->size().width(),m_fbo->size().height());
    glClearColor(m_background[0],m_background[1],m_background[2],1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    for(auto &m : m_objects) m->paint();

    glPixelStorei(GL_PACK_ALIGNMENT, (m_color.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH,m_color.step/m_color.elemSize());
    glReadPixels(render_rect.x,render_rect.y,render_rect.width,render_rect.height,GL_BGR,GL_UNSIGNED_BYTE,m_color.data);
    glPixelStorei(GL_PACK_ALIGNMENT,4);
    glPixelStorei(GL_PACK_ROW_LENGTH,m_depth.step/m_depth.elemSize());
    glReadPixels(render_rect.x,render_rect.y,render_rect.width,render_rect.height,GL_DEPTH_COMPONENT,GL_FLOAT,m_depth.data);
    convertZBufferToDepth(m_depth);

    m_fbo->release();
    doneCurrent();
}

void SingletonPainter::resizeGL(int w, int h){w=h=0;}

void SingletonPainter::clearBackground(float r,float g,float b)
{
    m_background << r,g,b;
}

void SingletonPainter::bindVBOs(vector<Vector3f> &vertex, vector<Vector3i> &faces, GLuint &vert, GLuint &ind)
{
    makeCurrent();
    glGenBuffers(1, &vert);
    glGenBuffers(1, &ind);

    glBindBuffer(GL_ARRAY_BUFFER, vert);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3f)*vertex.size(),vertex.data(),GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ind);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Vector3i)*faces.size(),faces.data(),GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    doneCurrent();
}


void SingletonPainter::drawVBOs(GLuint vert, GLuint ind, int count)
{
    glBindBuffer(GL_ARRAY_BUFFER, vert);
    glVertexPointer(3,GL_FLOAT,2*sizeof(Vector3f),0);
    glColorPointer (3,GL_FLOAT,2*sizeof(Vector3f),reinterpret_cast<void*>(sizeof(Vector3f)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ind);
    if (ind==0) glDrawArrays(GL_POINTS,0,count);
    else        glDrawElements(GL_TRIANGLES,count,GL_UNSIGNED_INT,0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void SingletonPainter::convertZBufferToDepth(Mat &depth)
{
	const float mult = (m_near*m_far) / (m_near - m_far);
	const float addi = m_far / (m_near - m_far);
	float *ptr = depth.ptr<float>(0);
	for (int i = 0; i < depth.rows*depth.cols; ++i, ++ptr)
		if (*ptr != 1.0) *ptr = mult / (*ptr + addi);
		else *ptr = 0;
}

SingletonPainter* Painter::m_singleton= 0;


RealWorldCamera::RealWorldCamera(Matrix3f &kma,Isometry3f &transform, float p_near, float p_far)
{
    m_near = p_near;
    m_far = p_far;
    m_pose = transform;
    m_cam = kma;
}


void RealWorldCamera::paint(void)
{
    //SPAAM paper of Nassir Navab...
    Matrix4f mat;
    mat.block(0,0,3,4) = m_cam*m_pose.affine();
    mat.block(3,0,1,4) = mat.block(2,0,1,4);
    mat.block(2,0,1,4) *= -(m_near+m_far); // scale factor
    mat(2,3) += m_near*m_far; //offset

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(viewport[0],viewport[2],viewport[1],viewport[3],m_near,m_far);
    glMultMatrixf(mat.data());
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}


BackgroundCamera::BackgroundCamera(Mat &background): m_background(background)
{
    assert(!m_background.empty());
}


void BackgroundCamera::paint(void)
{
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0,m_background.cols,0.0,m_background.rows,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPixelStorei(GL_UNPACK_ALIGNMENT,4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH,m_background.cols);
    glDrawPixels(m_background.cols,m_background.rows,GL_BGR,GL_UNSIGNED_BYTE,m_background.data);
}


CoordinateSystem::CoordinateSystem(float size): m_size(size)
{
    m_indices = {0,1,0,2,0,3};
    m_points.setZero();
    m_points(0,1) = m_points(1,2) = m_points(2,3) = m_size;
}


void CoordinateSystem::paint(void)
{
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glLineWidth(2);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3,GL_FLOAT,0,m_points.data());
    glColor3f(1.0f,0.0f,0.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_SHORT, &(m_indices[0]));
    glColor3f(0.0f,1.0f,0.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_SHORT, &(m_indices[2]));
    glColor3f(0.0f,0.0f,1.0f);
    glDrawElements(GL_LINES, 2, GL_UNSIGNED_SHORT, &(m_indices[4]));
    glDisableClientState(GL_VERTEX_ARRAY);
    glLineWidth(1);
}


BoundingBox::BoundingBox(Matrix<float,3,8> &in_bb): m_bb(in_bb)
{
    m_indices = {0, 1, 2, 3,4, 5, 6, 7,0, 4, 1, 5, 2, 6, 3, 7};
}


void BoundingBox::paint(void)
{
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glLineWidth(2);
    glColor3f(0.25,1,0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3,GL_FLOAT,0,m_bb.data());
    glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, &m_indices[0]);
    glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, &(m_indices[4]));
    glDrawElements(GL_LINES, 8, GL_UNSIGNED_SHORT, &(m_indices[8]));
    glDisableClientState(GL_VERTEX_ARRAY);
    glLineWidth(1);
}


AxisAlignedPlane::AxisAlignedPlane(int axis, float offset, float r, float g, float b)
{
    m_indices = {0, 1, 2, 2, 3, 1};

    switch(axis)
    {
    case 0:
        m_points.push_back(Vector3f(offset,-100, -100));
        m_points.push_back(Vector3f(offset,-100,  100));
        m_points.push_back(Vector3f(offset, 100, -100));
        m_points.push_back(Vector3f(offset, 100,  100));
        break;
    case 1:
        m_points.push_back(Vector3f(-100,offset, -100));
        m_points.push_back(Vector3f(-100,offset,  100));
        m_points.push_back(Vector3f( 100,offset, -100));
        m_points.push_back(Vector3f( 100,offset, 100));
        break;
    case 2:
        m_points.push_back(Vector3f(-100,-100, offset));
        m_points.push_back(Vector3f(-100, 100, offset));
        m_points.push_back(Vector3f( 100,-100, offset));
        m_points.push_back(Vector3f( 100, 100, offset));
        break;
    }

    m_r = r;
    m_g = g;
    m_b = b;

}


void AxisAlignedPlane::paint()
{
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    glColor3f(m_r,m_g,m_b);


    glBegin(GL_TRIANGLES);
    glVertex3f(m_points[0](0), m_points[0](1), m_points[0](2));
    glVertex3f(m_points[1](0), m_points[1](1), m_points[1](2));
    glVertex3f(m_points[2](0), m_points[2](1), m_points[2](2));
    glVertex3f(m_points[2](0), m_points[2](1), m_points[2](2));
    glVertex3f(m_points[3](0), m_points[3](1), m_points[3](2));
    glVertex3f(m_points[1](0), m_points[1](1), m_points[1](2));
    glEnd();

    /*
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(4,GL_FLOAT,0,m_points.data());
    glDrawElements(GL_TRIANGLES,1,GL_UNSIGNED_SHORT,m_indices.data());
    glDisableClientState(GL_VERTEX_ARRAY);
    */
}


