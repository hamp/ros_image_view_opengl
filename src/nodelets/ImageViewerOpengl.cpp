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
    printf("ImageViewerOpenGL constructor\n");
	reset();
}


ImageViewerOpenGL::~ImageViewerOpenGL()
{
    printf("ImageViewerOpenGL destructor\n");
}


void ImageViewerOpenGL::reset()
{
	setSceneRadius(80);
	setTextIsEnabled(false);
}

void ImageViewerOpenGL::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
//	meddleMutex.lock();

    printf("In ImageViewerOpenGL::addFrameMsg\n");

	if(!msg->isKeyframe)
	{
//		if(currentCamDisplay->id > msg->id)
//		{
//			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
//			resetRequested = true;
//		}
//		currentCamDisplay->setFrom(msg);
//		lastAnimTime = lastCamTime = msg->time;
//		lastCamID = msg->id;
        // copy over campose.
        memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
        
        fx = msg->fx;
        fy = msg->fy;
        cx = msg->cx;
        cy = msg->cy;
        
        fxi = 1/fx;
        fyi = 1/fy;
        cxi = -cx / fx;
        cyi = -cy / fy;
        
        width = msg->width;
        height = msg->height;
        id = msg->id;
        time = msg->time;
        
        printf("Cam translation: (%f, %f, %f)\n", camToWorld.translation().x(),
               camToWorld.translation().y(), camToWorld.translation().z());
//        glBuffersValid = false;

	}
//	else
//		graphDisplay->addMsg(msg);

//	meddleMutex.unlock();
}

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
    resize(640, 480);
    
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

GLuint texId = 0;

void ImageViewerOpenGL::drawFullScreenTex()
{
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    
//    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
//
//    glLoadIdentity();
//    // Render a color-cube consisting of 6 quads with different colors
//    //    glLoadIdentity();                 // Reset the model-view matrix
//    glPushMatrix();
//
//    int nWidth = 320;
//    int nHeight = 240;
//    glOrtho(0, nWidth, 0, nHeight, -1, 1);
//    
//    // Front facing texture
//    glBindTexture(GL_TEXTURE_2D, texId);
//
//    glBegin(GL_QUADS);
//    glTexCoord2f(0.0f,1.0f);
//    glVertex2i(0,nHeight-1);
//    glTexCoord2f(0.0f,0.0f);
//    glVertex2i(0,0);
//    glTexCoord2f(1.0f,0.0f);
//    glVertex2i(nWidth-1,0);
//    glTexCoord2f(1.0f,1.0f);
//    glVertex2i(nWidth-1,nHeight-1);
//    glEnd();
//    
//    // Free the texture memory
//    glDeleteTextures(1, &texId);
    
//    glPushAttrib( GL_TEXTURE_BIT | GL_DEPTH_TEST | GL_LIGHTING );
    
    glDisable( GL_DEPTH_TEST );
    glDisable( GL_LIGHTING );
    
    // Push back the current matrices and go orthographic for background rendering.
    glMatrixMode( GL_PROJECTION );
//    glPushMatrix();
    glLoadIdentity();
    glOrtho( 0, 640, 480, 0, -1, 1 );  //or whatever size you want
    
    glMatrixMode( GL_MODELVIEW );
//    glPushMatrix();
    glLoadIdentity();
    
    //build the background
    glBindTexture(GL_TEXTURE_2D, texId);		// Select Our Texture
    glBegin(GL_QUADS);				// Draw The Quad
    glColor3f( 1.0, 1.0, 1.0 );
    glTexCoord2f(0.0f, 0.0f); glVertex3f(0.0f, float(480), 0.0f);		// Bottom Left
    glTexCoord2f(1.0f, 0.0f); glVertex3f( float(640), float(480), 0.0f);				// Bottom Right
    glTexCoord2f(1.0f, 1.0f); glVertex3f( float(640), 0.0f, 0.0f);				// Top Right
    glTexCoord2f(0.0f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f);				// Top Left
    glEnd();										// Done Drawing The Quad
    
    
    // Pop everything back to what ever it was set to before we started
    
    glMatrixMode( GL_PROJECTION );
//    glPopMatrix();
    
    glMatrixMode( GL_MODELVIEW );
//    glPopMatrix();
    
//    glPopAttrib();
}

void ImageViewerOpenGL::draw()
{
//    printf("In ImageViewerOpenGL::draw()");
	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}

    if (_webCamMat != NULL)
    {
        texId = matToTexture(*_webCamMat, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
//        printf("new texId: %d \n", texId);
    }
    
//    drawFullScreenTex();
//    return;

    GLfloat cameraMatrix[16];
    camera()->getModelViewMatrix(cameraMatrix);
    printf("Cam translation: (%f, %f, %f)\n", cameraMatrix[12],
           cameraMatrix[13], cameraMatrix[14]);

//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // Render a color-cube consisting of 6 quads with different colors
//    glLoadIdentity();                 // Reset the model-view matrix
    glPushMatrix();
    glTranslatef(-cameraMatrix[12], cameraMatrix[13], cameraMatrix[14]);
    glTranslatef(0.f, 0.0f, -75.0f);  // Move right and into the screen
    
    glEnable(GL_TEXTURE_2D);
    
    // Quad width and height
    float w = 6.4f;
    float h = 4.8f;
    
    // Draw the textures
    // Note: Window co-ordinates origin is top left, texture co-ordinate origin is bottom left.
    
    // Front facing texture
    glBindTexture(GL_TEXTURE_2D, texId);
    
    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
//    glColor3f(1.0f, 1.0f, 1.0f);     // Green
//    glTexCoord2f(1, 1);
//    glVertex3f( 1.0f, 1.0f, -1.0f);
//    glTexCoord2f(0, 1);
//    glVertex3f(-1.0f, 1.0f, -1.0f);
//    glTexCoord2f(0, 0);
//    glVertex3f(-1.0f, 1.0f,  1.0f);
//    glTexCoord2f(1, 0);
//    glVertex3f( 1.0f, 1.0f,  1.0f);
//    
//    // Bottom face (y = -1.0f)
//    glColor3f(1.0f, 0.5f, 0.0f);     // Orange
//    glVertex3f( 1.0f, -1.0f,  1.0f);
//    glVertex3f(-1.0f, -1.0f,  1.0f);
//    glVertex3f(-1.0f, -1.0f, -1.0f);
//    glVertex3f( 1.0f, -1.0f, -1.0f);
    
    float a = 10.f;
    // Front face  (z = 1.0f)
    glColor3f(1.f, 1.f, 1.f);     // White
    glTexCoord2f(1, 0);
    glVertex3f( a*4,  a*3, a);
    glTexCoord2f(0, 0);
    glVertex3f(-a*4,  a*3, a);
    glTexCoord2f(0, 1);
    glVertex3f(-a*4, -a*3, a);
    glTexCoord2f(1, 1);
    glVertex3f( a*4, -a*3, a);
    
//    // Back face (z = -1.0f)
//    glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
//    glVertex3f( 1.0f, -1.0f, -1.0f);
//    glVertex3f(-1.0f, -1.0f, -1.0f);
//    glVertex3f(-1.0f,  1.0f, -1.0f);
//    glVertex3f( 1.0f,  1.0f, -1.0f);
//    
//    // Left face (x = -1.0f)
//    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
//    glVertex3f(-1.0f,  1.0f,  1.0f);
//    glVertex3f(-1.0f,  1.0f, -1.0f);
//    glVertex3f(-1.0f, -1.0f, -1.0f);
//    glVertex3f(-1.0f, -1.0f,  1.0f);
//    
//    // Right face (x = 1.0f)
//    glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
//    glVertex3f(1.0f,  1.0f, -1.0f);
//    glVertex3f(1.0f,  1.0f,  1.0f);
//    glVertex3f(1.0f, -1.0f,  1.0f);
//    glVertex3f(1.0f, -1.0f, -1.0f);

    glEnd();  // End of drawing color-cube
    glPopMatrix();

    
//    // Render a pyramid consists of 4 triangles
//    glPushMatrix();
//    glTranslatef(-1.5f, 0.0f, -6.0f);  // Move left and into the screen
//    glRotatef(_rotVal, 0.f, 1.f, 0.f);
//    
//    glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
//    // Front
//    glColor3f(1.0f, 0.0f, 0.0f);     // Red
//    glVertex3f( 0.0f, 1.0f, 0.0f);
//    glColor3f(0.0f, 1.0f, 0.0f);     // Green
//    glVertex3f(-1.0f, -1.0f, 1.0f);
//    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
//    glVertex3f(1.0f, -1.0f, 1.0f);
//    
//    // Right
//    glColor3f(1.0f, 0.0f, 0.0f);     // Red
//    glVertex3f(0.0f, 1.0f, 0.0f);
//    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
//    glVertex3f(1.0f, -1.0f, 1.0f);
//    glColor3f(0.0f, 1.0f, 0.0f);     // Green
//    glVertex3f(1.0f, -1.0f, -1.0f);
//    
//    // Back
//    glColor3f(1.0f, 0.0f, 0.0f);     // Red
//    glVertex3f(0.0f, 1.0f, 0.0f);
//    glColor3f(0.0f, 1.0f, 0.0f);     // Green
//    glVertex3f(1.0f, -1.0f, -1.0f);
//    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
//    glVertex3f(-1.0f, -1.0f, -1.0f);
//    
//    // Left
//    glColor3f(1.0f,0.0f,0.0f);       // Red
//    glVertex3f( 0.0f, 1.0f, 0.0f);
//    glColor3f(0.0f,0.0f,1.0f);       // Blue
//    glVertex3f(-1.0f,-1.0f,-1.0f);
//    glColor3f(0.0f,1.0f,0.0f);       // Green
//    glVertex3f(-1.0f,-1.0f, 1.0f);
//    glEnd();   // Done drawing the pyramid
//    glPopMatrix();
    
    glDisable(GL_TEXTURE_2D);

    // Free the texture memory
    glDeleteTextures(1, &texId);

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

GLuint ImageViewerOpenGL::matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);
    
    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);
    
    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
        magFilter == GL_LINEAR_MIPMAP_NEAREST ||
        magFilter == GL_NEAREST_MIPMAP_LINEAR ||
        magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        std::cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << std::endl;
        magFilter = GL_LINEAR;
    }
    
    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
    
    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);
    
    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_LUMINANCE;
    }
    
    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself
    
    // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
    if (minFilter == GL_LINEAR_MIPMAP_LINEAR  ||
        minFilter == GL_LINEAR_MIPMAP_NEAREST ||
        minFilter == GL_NEAREST_MIPMAP_LINEAR ||
        minFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    
    return textureID;
}

void ImageViewerOpenGL::SetWebcamMat(cv::Mat *mat)
{
    _webCamMat = mat;
}

