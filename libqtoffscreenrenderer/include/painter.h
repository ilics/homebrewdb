//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef PAINTER_H
#define PAINTER_H

#include <QtOpenGL>
#include <QOpenGLFunctions>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <opencv2/core.hpp>


// using namespace cv;
// using namespace Eigen;
using namespace std;


class PaintObject
{
public:
    virtual void paint(void)=0;
};


class SingletonPainter : public QGLWidget, protected QOpenGLFunctions
{
public:
	
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SingletonPainter(float p_near,float p_far,int width,int height);
    ~SingletonPainter();

    int getHeight(){return m_fbo->size().height();}
    int getWidth(){return m_fbo->size().width();}
    float getNear(){return m_near;}
    float getFar(){return m_far;}

    void clearBackground(float r,float g,float b);
    void clearObjects(){m_objects.clear();}

    void addPaintObject(PaintObject *object){m_objects.push_back(object);}
	
    void paint(int x=0,int y=0,int w=0,int h=0);

    void bindVBOs(vector<Eigen::Vector3f> &vertex_data, vector<Eigen::Vector3i> &faces_data, GLuint &vert, GLuint &ind);
    void drawVBOs(GLuint vert, GLuint ind, int count);

    inline void copyColorTo(cv::Mat &dest){m_color(copy_rect).copyTo(dest);}
    inline void copyDepthTo(cv::Mat &dest){m_depth(copy_rect).copyTo(dest);}

protected:
    
    void convertZBufferToDepth(cv::Mat &depth);
    void resizeGL(int w,int h);
	void paintGL();

private:
    
    // Rect in the image which should be rendered.
	cv::Rect render_rect, copy_rect;
    //OpenGL near/far.
    float m_near,m_far;
    //Background RGB color
	Eigen::Vector3f m_background;
    //The rendered depth and color buffer is stored here.
	cv::Mat m_depth,m_color;
    //Vector of objects that is painted (in this order).
    vector<PaintObject*> m_objects;
    //QT framebuffer object for offline rendering.
    QOpenGLFramebufferObject *m_fbo;
};


class Painter : protected QOpenGLFunctions
{

public:
	
    Painter()
    {}

    ~Painter(){}

    int getHeight(){return getSingleton()->getHeight();}
    int getWidth(){return getSingleton()->getWidth();}
    float getNear(){return getSingleton()->getNear();}
    float getFar(){return getSingleton()->getFar();}
    inline float getAspect(){return static_cast<float>(getWidth())/static_cast<float>(getHeight());}

    void setBackground(Eigen::Vector3f &col){getSingleton()->clearBackground(col(0),col(1),col(2));}
    void setBackground(float r,float g,float b){getSingleton()->clearBackground(r,g,b);}
    void clearObjects(){getSingleton()->clearObjects();}

    inline void addPaintObject(PaintObject *object){getSingleton()->addPaintObject(object);}
	
    inline void paint(int x=0,int y=0,int w=0,int h=0){getSingleton()->paint(x,y,w,h);}

    inline void copyColorTo(cv::Mat &dest){getSingleton()->copyColorTo(dest);}
    inline void copyDepthTo(cv::Mat &dest){getSingleton()->copyDepthTo(dest);}

	static float z_near;
	static float z_far;
	
	static int width;
	static int height;

	static SingletonPainter* getSingleton()
	{
		if (m_singleton == 0)
		{
			int argc = 0;
			char **argv = 0;
			if (!QApplication::instance()) new QApplication(argc, argv);
			if (!QGLFormat::hasOpenGL() || !QGLFramebufferObject::hasOpenGLFramebufferObjects())
			{
				cerr << "OpenGL error: No support of OpenGL/framebuffer objects." << endl;
				exit(0);
			}
			m_singleton = new SingletonPainter(z_near, z_far, width, height);

		}
		return m_singleton;
	}

private:
	
    //We need only ony QT framebuffer object in which we paint
    static SingletonPainter *m_singleton;
	
};


class RealWorldCamera : public PaintObject
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual ~RealWorldCamera(){}
    RealWorldCamera(Eigen::Matrix3f &kma, Eigen::Isometry3f &transform, float p_near, float p_far);
    void paint();
    
    // OpenGL near/far
    float m_near,m_far;
    // Rotation and translation matrix.
	Eigen::Isometry3f m_pose;
    // Internal camera matrix.
	Eigen::Matrix3f m_cam;
};


class BackgroundCamera : public PaintObject
{
public:

    BackgroundCamera(cv::Mat &background);

    void paint();

	cv::Mat m_background; //This image stores the background image.
};

class CoordinateSystem : public PaintObject
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CoordinateSystem(float size);

    void paint();

    float m_size;  // Size of the coordinate axis
	Eigen::Matrix<float,3,4> m_points;
    vector<GLushort> m_indices;

};

class AxisAlignedPlane : public PaintObject
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AxisAlignedPlane(int axis, float offset, float r,float g, float b);

    void paint();

    float m_r, m_g, m_b;    // Color of plane
    vector<Eigen::Vector3f> m_points;
    vector<GLushort> m_indices;

};


class BoundingBox : public PaintObject
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BoundingBox(Eigen::Matrix<float,3,8> &in_bb);
    virtual ~BoundingBox(){}

    void paint();

	Eigen::Matrix<float,3,8> m_bb;  // Bounding box
    vector<GLushort> m_indices;

};

#endif
