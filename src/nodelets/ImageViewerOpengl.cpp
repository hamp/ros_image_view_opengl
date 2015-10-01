/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#define GL_GLEXT_PROTOTYPES 1
#include "ImageViewerOpenGL.h"
#include "qfiledialog.h"
#include "qcoreapplication.h"
#include <stdio.h>

#include <zlib.h>
#include <iostream>


//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include "QGLViewer/manipulatedCameraFrame.h"

#include <iostream>
#include <fstream>

ImageViewerOpenGL::ImageViewerOpenGL()
{
    printf("ImageViewerOpenGL constructor");
	reset();
}


ImageViewerOpenGL::~ImageViewerOpenGL()
{
    printf("ImageViewerOpenGL destructor");
}


void ImageViewerOpenGL::reset()
{
	setSceneRadius(80);
	setTextIsEnabled(false);
}

//void ImageViewerOpenGL::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
//{
//	meddleMutex.lock();
//
//	if(!msg->isKeyframe)
//	{
//		if(currentCamDisplay->id > msg->id)
//		{
//			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
//			resetRequested = true;
//		}
//		currentCamDisplay->setFrom(msg);
//		lastAnimTime = lastCamTime = msg->time;
//		lastCamID = msg->id;
//	}
//	else
//		graphDisplay->addMsg(msg);
//
//	meddleMutex.unlock();
//}
//
//void ImageViewerOpenGL::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
//{
//	meddleMutex.lock();
//
//	graphDisplay->addGraphMsg(msg);
//
//	meddleMutex.unlock();
//}


void ImageViewerOpenGL::init()
{
	setAnimationPeriod(30);
	startAnimation();
    
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);                   // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

}

QString ImageViewerOpenGL::helpString() const
{
	return QString("");
}


void ImageViewerOpenGL::draw()
{
//    printf("In ImageViewerOpenGL::draw()");
	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}


//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    
    // Render a color-cube consisting of 6 quads with different colors
//    glLoadIdentity();                 // Reset the model-view matrix
    glPushMatrix();
    glTranslatef(1.5f, 0.0f, -7.0f);  // Move right and into the screen
    
    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f( 1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f, -1.0f);
    glVertex3f(-1.0f, 1.0f,  1.0f);
    glVertex3f( 1.0f, 1.0f,  1.0f);
    
    // Bottom face (y = -1.0f)
    glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f( 1.0f, -1.0f,  1.0f);
    glVertex3f(-1.0f, -1.0f,  1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f( 1.0f, -1.0f, -1.0f);
    
    // Front face  (z = 1.0f)
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f( 1.0f,  1.0f, 1.0f);
    glVertex3f(-1.0f,  1.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glVertex3f( 1.0f, -1.0f, 1.0f);
    
    // Back face (z = -1.0f)
    glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f( 1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f,  1.0f, -1.0f);
    glVertex3f( 1.0f,  1.0f, -1.0f);
    
    // Left face (x = -1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-1.0f,  1.0f,  1.0f);
    glVertex3f(-1.0f,  1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f);
    glVertex3f(-1.0f, -1.0f,  1.0f);
    
    // Right face (x = 1.0f)
    glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(1.0f,  1.0f, -1.0f);
    glVertex3f(1.0f,  1.0f,  1.0f);
    glVertex3f(1.0f, -1.0f,  1.0f);
    glVertex3f(1.0f, -1.0f, -1.0f);
    glEnd();  // End of drawing color-cube
    glPopMatrix();

    
    // Render a pyramid consists of 4 triangles
    glPushMatrix();
    glTranslatef(-1.5f, 0.0f, -6.0f);  // Move left and into the screen
    glRotatef(_rotVal, 0.f, 1.f, 0.f);
    
    glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
    // Front
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f( 0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(-1.0f, -1.0f, 1.0f);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(1.0f, -1.0f, 1.0f);
    
    // Right
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(1.0f, -1.0f, 1.0f);
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(1.0f, -1.0f, -1.0f);
    
    // Back
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(1.0f, -1.0f, -1.0f);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-1.0f, -1.0f, -1.0f);
    
    // Left
    glColor3f(1.0f,0.0f,0.0f);       // Red
    glVertex3f( 0.0f, 1.0f, 0.0f);
    glColor3f(0.0f,0.0f,1.0f);       // Blue
    glVertex3f(-1.0f,-1.0f,-1.0f);
    glColor3f(0.0f,1.0f,0.0f);       // Green
    glVertex3f(-1.0f,-1.0f, 1.0f);
    glEnd();   // Done drawing the pyramid
    glPopMatrix();
}

void ImageViewerOpenGL::keyReleaseEvent(QKeyEvent *e)
{

}

void ImageViewerOpenGL::setToVideoSize()
{
	this->setFixedSize(1600,900);
}

void ImageViewerOpenGL::keyPressEvent(QKeyEvent *e)
  {
    switch (e->key())
    {
      case Qt::Key_S :
    	    setToVideoSize();
    	  break;

      case Qt::Key_R :
    	    resetRequested = true;

    	  break;

      case Qt::Key_T:	// add settings item

    	  break;

      case Qt::Key_K:	// add keyframe item
    	  break;

      case Qt::Key_I :	// reset animation list
			printf("resetted animation list!\n");

    	  break;


      case Qt::Key_F :	// save list
      {
			printf("saved animation list (%d items)!\n", 0);
      }
    	  break;


      case Qt::Key_L :	// load list
      {
			printf("loaded animation list! (%d items)!\n", 0);
      }
    	  break;


      case Qt::Key_A:
    		  printf("ENABLE custom animation!\n");
    	  break;

      case Qt::Key_O:
      	  break;


      case Qt::Key_P:
    	  break;

      case Qt::Key_W:
    	  break;

      default:
    	  QGLViewer::keyPressEvent(e);
    	  break;
    }
  }

void ImageViewerOpenGL::SetRotTest(float value)
{
    _rotVal = value;
//    printf ("rotval: %f", _rotVal);
}

