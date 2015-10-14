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

// assimp include files. These three are usually needed.
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)


// the global Assimp scene object
const struct aiScene* scene = NULL;
GLuint scene_list = 0;
struct aiVector3t<float> scene_min, scene_max, scene_center;


// ----------------------------------------------------------------------------
void get_bounding_box_for_node (const struct aiNode* nd,
                                struct aiVector3t<float>* min,
                                struct aiVector3t<float>* max,
                                struct aiMatrix4x4t<float>* trafo
                                ){
    struct aiMatrix4x4t<float> prev;
    unsigned int n = 0, t;
    
    prev = *trafo;
    aiMultiplyMatrix4(trafo,&nd->mTransformation);
    
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
        for (t = 0; t < mesh->mNumVertices; ++t) {
            
            struct aiVector3t<float> tmp = mesh->mVertices[t];
            aiTransformVecByMatrix4(&tmp,trafo);
            
            min->x = aisgl_min(min->x,tmp.x);
            min->y = aisgl_min(min->y,tmp.y);
            min->z = aisgl_min(min->z,tmp.z);
            
            max->x = aisgl_max(max->x,tmp.x);
            max->y = aisgl_max(max->y,tmp.y);
            max->z = aisgl_max(max->z,tmp.z);
        }
    }
    
    for (n = 0; n < nd->mNumChildren; ++n) {
        get_bounding_box_for_node(nd->mChildren[n],min,max,trafo);
    }
    *trafo = prev;
}

// ----------------------------------------------------------------------------
void get_bounding_box (struct aiVector3t<float>* min, struct aiVector3t<float>* max)
{
    struct aiMatrix4x4t<float> trafo;
    aiIdentityMatrix4(&trafo);
    
    min->x = min->y = min->z =  1e10f;
    max->x = max->y = max->z = -1e10f;
    get_bounding_box_for_node(scene->mRootNode,min,max,&trafo);
}



// ----------------------------------------------------------------------------
int loadasset (const char* path)
{
    // we are taking one of the postprocessing presets to avoid
    // spelling out 20+ single postprocessing flags here.
    scene = aiImportFile(path,aiProcessPreset_TargetRealtime_MaxQuality);
    
    if (scene) {
        get_bounding_box(&scene_min,&scene_max);
        scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
        scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
        scene_center.z = (scene_min.z + scene_max.z) / 2.0f;
        return 0;
    }
    return 1;
}

// ----------------------------------------------------------------------------
void color4_to_float4(const struct aiColor4t<float> *c, float f[4])
{
    f[0] = c->r;
    f[1] = c->g;
    f[2] = c->b;
    f[3] = c->a;
}

// ----------------------------------------------------------------------------
void set_float4(float f[4], float a, float b, float c, float d)
{
    f[0] = a;
    f[1] = b;
    f[2] = c;
    f[3] = d;
}

// ----------------------------------------------------------------------------
void apply_material(const struct aiMaterial *mtl)
{
    float c[4];
    
    GLenum fill_mode;
    int ret1, ret2;
    struct aiColor4t<float> diffuse;
    struct aiColor4t<float> specular;
    struct aiColor4t<float> ambient;
    struct aiColor4t<float> emission;
    float shininess, strength;
    int two_sided;
    int wireframe;
    unsigned int max;
    
    set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        color4_to_float4(&diffuse, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);
    
    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
        color4_to_float4(&specular, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
    
    set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
        color4_to_float4(&ambient, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);
    
    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
        color4_to_float4(&emission, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);
    
    max = 1;
    ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
    if(ret1 == AI_SUCCESS) {
        max = 1;
        ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
        if(ret2 == AI_SUCCESS)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
    else {
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
        set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
    }
    
    max = 1;
    if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
        fill_mode = wireframe ? GL_LINE : GL_FILL;
    else
        fill_mode = GL_FILL;
    glPolygonMode(GL_FRONT_AND_BACK, fill_mode);
    
    max = 1;
    if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
        glDisable(GL_CULL_FACE);
    else
        glEnable(GL_CULL_FACE);
}

// ----------------------------------------------------------------------------
void recursive_render (const struct aiScene *sc, const struct aiNode* nd)
{
    unsigned int i;
    unsigned int n = 0, t;
    struct aiMatrix4x4t<float> m = nd->mTransformation;
    
    // update transform
    aiTransposeMatrix4(&m);
    glPushMatrix();
    glMultMatrixf((float*)&m);
    
    // draw all meshes assigned to this node
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
        
        apply_material(sc->mMaterials[mesh->mMaterialIndex]);
        
        if(mesh->mNormals == NULL) {
            glDisable(GL_LIGHTING);
        } else {
            glEnable(GL_LIGHTING);
        }
        
        for (t = 0; t < mesh->mNumFaces; ++t) {
            const struct aiFace* face = &mesh->mFaces[t];
            GLenum face_mode;
            
            switch(face->mNumIndices) {
                case 1: face_mode = GL_POINTS; break;
                case 2: face_mode = GL_LINES; break;
                case 3: face_mode = GL_TRIANGLES; break;
                default: face_mode = GL_POLYGON; break;
            }
            
            glBegin(face_mode);
            
            for(i = 0; i < face->mNumIndices; i++) {
                int index = face->mIndices[i];
                if(mesh->mColors[0] != NULL)
                    glColor4fv((GLfloat*)&mesh->mColors[0][index]);
                if(mesh->mNormals != NULL) 
                    glNormal3fv(&mesh->mNormals[index].x);
                glVertex3fv(&mesh->mVertices[index].x);
            }
            
            glEnd();
        }
        
    }
    
    // draw all children
    for (n = 0; n < nd->mNumChildren; ++n) {
        recursive_render(sc, nd->mChildren[n]);
    }
    
    glPopMatrix();
}



ImageViewerOpenGL::ImageViewerOpenGL()
{
    printf("ImageViewerOpenGL constructor\n");
	reset();
    
    // get a handle to the predefined STDOUT log stream and attach
    // it to the logging system. It remains active for all further
    // calls to aiImportFile(Ex) and aiApplyPostProcessing.
    struct aiLogStream stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
    aiAttachLogStream(&stream);
    
    // ... same procedure, but this stream now writes the
    // log messages to assimp_log.txt
    stream = aiGetPredefinedLogStream(aiDefaultLogStream_FILE,"assimp_log.txt");
    aiAttachLogStream(&stream);
    
    // the model name can be specified on the command line. If none
    // is specified, we try to locate one of the more expressive test
    // models from the repository (/models-nonbsd may be missing in
    // some distributions so we need a fallback from /models!).
    if( 0 != loadasset( "/Users/kaushikdas/Downloads/f209638111b194de38731945a6e74476/Chair.3DS")) {
        if( (0 != loadasset( "../../../../test/models-nonbsd/X/dwarf.x") && 0 != loadasset( "../../test/models/X/Testwuson.X"))) {
            printf("could not load model");
        }
    }

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

GLfloat diffuseMaterial[4] = { 0.5, 0.5, 0.5, 1.0 };

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
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);    /* Uses default lighting parameters */
    
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_NORMALIZE);
    
    /* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
    if(getenv("MODEL_IS_BROKEN"))
        glFrontFace(GL_CW);
    
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    camera()->setFieldOfView(70.f * 3.1416 / 180.f);
    camera()->setZNearCoefficient(0.001f);
}

QString ImageViewerOpenGL::helpString() const
{
	return QString("");
}

GLuint texId = 0;

void ImageViewerOpenGL::drawFullScreenTex()
{
    //TODO:
    float tmp;
    
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
//    gluLookAt(0.f,0.f,3.f,0.f,0.f,-5.f,0.f,1.f,0.f);
//    
    // rotate it around the y axis
    glRotatef(_rotVal,0.f,1.f,0.f);

    // scale the whole asset to fit into our view frustum
    tmp = scene_max.x-scene_min.x;
    tmp = aisgl_max(scene_max.y - scene_min.y,tmp);
    tmp = aisgl_max(scene_max.z - scene_min.z,tmp);
    tmp = 1.f / tmp;
    glScalef(tmp, tmp, tmp);
    
//    // center the model
//    glTranslatef( -scene_center.x, -scene_center.y, -scene_center.z );
    
    // if the display list has not been made yet, create a new one and
    // fill it with scene contents
    if(scene_list == 0) {
        scene_list = glGenLists(1);
        glNewList(scene_list, GL_COMPILE);
        // now begin at the root node of the imported data and traverse
        // the scenegraph by multiplying subsequent local transforms
        // together on GL's matrix stack.
        recursive_render(scene, scene->mRootNode);
        glEndList();
    }
    
    glCallList(scene_list);
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

    //Fixing camera movement, keeping it at origin
    GLdouble cameraMatrix[16];
//    camera()->getModelViewMatrix(cameraMatrix);
//    printf("Cam translation: (%f, %f, %f)\n", cameraMatrix[12],
//           cameraMatrix[13], cameraMatrix[14]);
    
    cameraMatrix[0] = cameraMatrix[5] = cameraMatrix[10] = cameraMatrix[15] = 1.f;
    cameraMatrix[1] = cameraMatrix[2] = cameraMatrix[3] = cameraMatrix[6] = cameraMatrix[7] = cameraMatrix[8] = cameraMatrix[9] = 0.f;
    cameraMatrix[11] = 0.f;
    cameraMatrix[12] = 0.f;
    cameraMatrix[13] = 0.f;
    cameraMatrix[14] = 0.f;

    camera()->setFromModelViewMatrix(cameraMatrix);
    
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // Render a color-cube consisting of 6 quads with different colors
//    glLoadIdentity();                 // Reset the model-view matrix
//    glPushMatrix();
    glLoadIdentity();
//    glTranslated(-cameraMatrix[12], cameraMatrix[13], cameraMatrix[14]);
    glTranslatef(0.f, 0.0f, -52.0f);
    
    glEnable(GL_TEXTURE_2D);
    // Draw the webcam texture
    // Note: Window co-ordinates origin is top left, texture co-ordinate origin is bottom left.
    
    // Front facing texture
    glBindTexture(GL_TEXTURE_2D, texId);
    
    glBegin(GL_QUADS);                // Begin drawing the webcam texture
    float a = 10.f;
    // Front face  (z = 1.0f)
    glColor3f(1.f, 1.f, 1.f);     // White
    glTexCoord2f(1, 0);
    glVertex3f( a*4,  a*3, a);//Expecting 4:3 aspect ratio
    glTexCoord2f(0, 0);
    glVertex3f(-a*4,  a*3, a);
    glTexCoord2f(0, 1);
    glVertex3f(-a*4, -a*3, a);
    glTexCoord2f(1, 1);
    glVertex3f( a*4, -a*3, a);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    
//    // Free the texture memory
//    glDeleteTextures(1, &texId);

//    glPopMatrix();
    
    
    //Draw object
    glPushMatrix();
    glLoadIdentity();
//    glLoadMatrixd(cameraMatrix);
    
    float translationOffset[] = {0.f, -1.f, -1.5f};
    translationOffset[0] -= camToWorld.translation().x()*1.f;
    translationOffset[1] += camToWorld.translation().y()*1.f;
    translationOffset[2] += camToWorld.translation().z()*1.f;

    glRotatef(35.f, 1.f, 0.f, 0.f);
    glTranslatef(translationOffset[0], translationOffset[1], translationOffset[2]);

    drawFullScreenTex();
    
    //Local Rot
    glRotatef(33.f, 0.f, 1.f, 0.f);

    float cubeWidth = 0.5f;
    float cubeHeight = 0.5f;
    float cubeDepth = 0.5f;
    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    glColor3f(0.9f, 0.9f, 0.9f);     // Green
    glVertex3f( cubeWidth, cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth, cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth, cubeHeight,  cubeDepth);
    glVertex3f( cubeWidth, cubeHeight,  cubeDepth);
    
    // Bottom face (y = -1.0f)
    glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f( cubeWidth, -cubeHeight,  cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight,  cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight, -cubeDepth);
    glVertex3f( cubeWidth, -cubeHeight, -cubeDepth);
    
    // Front face  (z = 1.0f)
    glColor3f(0.5f, 0.5f, 0.5f);     // White
    glVertex3f( cubeWidth,  cubeHeight, cubeDepth);
    glVertex3f(-cubeWidth,  cubeHeight, cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight, cubeDepth);
    glVertex3f(cubeWidth, -cubeHeight, cubeDepth);
    
    // Back face (z = -1.0f)
    glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f( cubeWidth, -cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth,  cubeHeight, -cubeDepth);
    glVertex3f( cubeWidth,  cubeHeight, -cubeDepth);

    // Left face (x = -1.0f)
    glColor3f(1.0f, 1.0f, 1.0f);     // Blue
    glVertex3f(-cubeWidth,  cubeHeight,  cubeDepth);
    glVertex3f(-cubeWidth,  cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight, -cubeDepth);
    glVertex3f(-cubeWidth, -cubeHeight,  cubeDepth);
    
    // Right face (x = 1.0f)
    glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(cubeWidth,  cubeHeight, -cubeDepth);
    glVertex3f(cubeWidth,  cubeHeight,  cubeDepth);
    glVertex3f(cubeWidth, -cubeHeight,  cubeDepth);
    glVertex3f(cubeWidth, -cubeHeight, -cubeDepth);

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

