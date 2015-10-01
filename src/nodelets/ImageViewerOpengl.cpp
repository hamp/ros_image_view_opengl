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
	reset();
}


ImageViewerOpenGL::~ImageViewerOpenGL()
{
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
}

QString ImageViewerOpenGL::helpString() const
{
	return QString("");
}

void ImageViewerOpenGL::draw()
{
	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}


	glPushMatrix();

//	graphDisplay->draw();

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

