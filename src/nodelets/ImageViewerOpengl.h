#pragma once
//#define GL_GLEXT_PROTOTYPES 1
//#define GL3_PROTOTYPES 1
//#include <GL/glew.h>

#include "QGLViewer/qglviewer.h"
#include <vector>
#include "boost/thread.hpp"
#include "qevent.h"

#include <opencv2/opencv.hpp>

#include "QGLViewer/keyFrameInterpolator.h"

class QApplication;


class ImageViewerOpenGL : public QGLViewer
{
public:
	ImageViewerOpenGL();
	~ImageViewerOpenGL();


	void reset();

//	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
//	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);

    void SetRotTest(float value);
    void SetWebcamMat(cv::Mat *mat);

protected :
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void keyReleaseEvent(QKeyEvent *e);
	virtual QString helpString() const;

//	virtual void drawText(int x, int y, const QString & text, const QFont & fnt) {printf(text.toStdString().c_str());};

    GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);

private:


	void setToVideoSize();
	bool resetRequested;
    
    float _rotVal;
    cv::Mat *_webCamMat;
};


