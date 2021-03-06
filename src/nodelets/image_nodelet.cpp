/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"

#include <boost/thread.hpp>
#include <boost/format.hpp>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

#include "ImageViewerOpengl.h"
#include <qapplication.h>

ImageViewerOpenGL *viewer;
QApplication *application;

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroyNode(GtkWidget *widget, gpointer data)
{
  /// @todo On ros::shutdown(), the node hangs. Why?
  //ros::shutdown();
  exit(0); // brute force solution
}

static void destroyNodelet(GtkWidget *widget, gpointer data)
{
  // We can't actually unload the nodelet from here, but we can at least
  // unsubscribe from the image topic.
  reinterpret_cast<image_transport::Subscriber*>(data)->shutdown();
}
#endif


namespace image_view_opengl {

class ImageNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_;

  boost::mutex image_mutex_;
  cv::Mat last_image_;
  
  std::string window_name_;
  boost::format filename_format_;
  int count_;

  virtual void onInit();
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  static void onOpenGlDraw(void* param);

  static void mouseCb(int event, int x, int y, int flags, void* param);

//  static void startQGLThread();
public:
  ImageNodelet();

  ~ImageNodelet();
};

ImageNodelet::ImageNodelet()
  : filename_format_(""), count_(0)
{
}

ImageNodelet::~ImageNodelet()
{
  cv::destroyWindow(window_name_);
}

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
    printf("In frameCb\n");
   if(viewer != 0)
       viewer->addFrameMsg(msg);
}
    
void rosThreadFn()
{
    printf("In frosThreadFn\n");
    ros::NodeHandle nh2;
    
    ros::Subscriber liveFrames_sub = nh2.subscribe(nh2.resolveName("lsd_slam/liveframes"),1, frameCb);
    ros::Subscriber keyFrames_sub = nh2.subscribe(nh2.resolveName("lsd_slam/keyframes"),20, frameCb);

    ros::spin();
    
    ros::shutdown();
    
    printf("Exiting ROS thread\n");
    
    exit(1);
}

void ImageNodelet::onInit()
{
  std::cerr << "checking if here" << std::endl;
    
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();

  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport;
  local_nh.param("image_transport", transport, std::string("raw"));
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
 
  NODELET_INFO_STREAM("Using transport \"" << transport << "\"");
  
  // Internal option, should be used only by the image_view node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  bool autosize;
  local_nh.param("autosize", autosize, false);
  
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  cv::namedWindow(window_name_, autosize ? cv::WND_PROP_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &ImageNodelet::mouseCb, this);

  NODELET_INFO_STREAM("Window name: \"" << window_name_ << "\"");
    
//  cv::setOpenGlDrawCallback(window_name_, &ImageNodelet::onOpenGlDraw, NULL);
#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
  else
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_);
#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), getPrivateNodeHandle());
  sub_ = it.subscribe(topic, 1, &ImageNodelet::imageCb, this, hints);
  
   
    printf("starting QGL thread\n");
    
    // Read command lines arguments.
    int argcdummy = 1;
    char *argvdummy = "dummy";
    application = new QApplication (argcdummy, &argvdummy);
    
    // Instantiate the viewer.
    viewer = new ImageViewerOpenGL();
    
#if QT_VERSION < 0x040000
    // Set the viewer as the application main widget.
    application->setMainWidget(viewer);
#else
    viewer->setWindowTitle("OpenGL Viewer");
#endif
    
    // Make the viewer window visible on screen.
    viewer->show();

    boost::thread rosThread(rosThreadFn);
    
  application->exec();
}
    

void ImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  image_mutex_.lock();

  // We want to scale floating point images so that they display nicely
  if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if(max_val > 0)
      last_image_ = float_image / max_val;
    else
      last_image_ = float_image.clone();
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8: '%s'",
                             msg->encoding.c_str(), e.what());
    }
  }

//    cv::line(last_image_, cv::Point2f(0.f, 0.f), cv::Point2f(100.f, 100.f), cv::Scalar(1.f, 0.f, 0.f, 1.f)) ;
//    cv::circle(last_image_, cv::Point2f(0.f, 0.f), 20, cv::Scalar(1.f, 1.f, 1.f, 1.f), 5) ;
//    std::stringstream ss;
//    ss << "img size: " << last_image_.cols << "x" << last_image_.rows << std::endl;
//    ss << "img size: " << last_image_.cols << "x" << last_image_.rows << std::endl;
//    NODELET_INFO_STREAM(ss.str().c_str());
    
  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  image_mutex_.unlock();
  if (!last_image_.empty())
    cv::imshow(window_name_, last_image_);
    
    static int cntr = 0;
    if (viewer != NULL)
    {
        viewer->SetRotTest(cntr++*10.f);
        viewer->SetWebcamMat(&last_image_);
    }
}

void ImageNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  ImageNodelet *this_ = reinterpret_cast<ImageNodelet*>(param);
  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&ImageNodelet::getName, this_);

  if (event == cv::EVENT_LBUTTONDOWN)
  {
    NODELET_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    return;
  }
  if (event != cv::EVENT_RBUTTONDOWN)
    return;
  
  boost::lock_guard<boost::mutex> guard(this_->image_mutex_);

  const cv::Mat &image = this_->last_image_;
  if (image.empty())
  {
    NODELET_WARN("Couldn't save image, no data!");
    return;
  }

  std::string filename = (this_->filename_format_ % this_->count_).str();
  if (cv::imwrite(filename, image))
  {
    NODELET_INFO("Saved image %s", filename.c_str());
    this_->count_++;
  }
  else
  {
    /// @todo Show full path, ask if user has permission to write there
    NODELET_ERROR("Failed to save image.");
  }
}

void ImageNodelet::onOpenGlDraw(void* param)
{
    std::cout << "In OpenGL draw" << std::endl;
}
                            
} // namespace image_view

                            
// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_view_opengl::ImageNodelet, nodelet::Nodelet)
