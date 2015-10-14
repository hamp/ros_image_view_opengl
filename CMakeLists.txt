cmake_minimum_required(VERSION 2.8)
project(image_view_opengl)

find_package(catkin REQUIRED)
catkin_package()

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  add_definitions("-DOPENCV3=1")
endif()

find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport message_filters nodelet rosconsole roscpp)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
			   ./include
)

#For OpenGL/QGL
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)

find_package(QGLViewer REQUIRED)
find_package(Eigen3 REQUIRED)

set(Assimp_SOURCE_DIR /Users/kaushikdas/Work/Development/assimp-3.1.1)

# Extra tools
add_executable(extract_images src/nodes/extract_images.cpp)
target_link_libraries(extract_images ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(image_saver src/nodes/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)

add_executable(video_recorder src/nodes/video_recorder.cpp)
target_link_libraries(video_recorder ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

install(TARGETS extract_images image_saver video_recorder
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(SYSTEM ${GTK2_INCLUDE_DIRS}
			  ${QGLVIEWER_INCLUDE_DIR}
			  /usr/local/include/eigen3
			  ${EIGEN_INCLUDE_DIR}
	${Assimp_SOURCE_DIR}/include
	${Assimp_SOURCE_DIR}/code
)

# Nodelet library
add_library(image_view_opengl src/nodelets/image_nodelet.cpp src/nodelets/disparity_nodelet.cpp src/nodelets/window_thread.cpp src/nodelets/ImageViewerOpengl.cpp)
target_link_libraries(image_view_opengl ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
				 ${QT_LIBRARIES}
				 ${QGLViewer_LIBRARIES}
			         ${QGLVIEWER_LIBRARY}
				 ${OpenGL_LIBRARIES}
			     	 ${OPENGL_LIBRARY}
				 /usr/local/Cellar/glew/1.12.0/lib/libGLEW.dylib
				/Users/kaushikdas/Work/Development/assimp-3.1.1/build/code/libassimp.dylib
			     
)
install(TARGETS image_view_opengl
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Image viewers
add_executable(image_view_opengl_exe src/nodes/image_view.cpp)
SET_TARGET_PROPERTIES(image_view_opengl_exe PROPERTIES OUTPUT_NAME image_view_opengl)
target_link_libraries(image_view_opengl_exe ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(disparity_view src/nodes/disparity_view.cpp)
target_link_libraries(disparity_view ${catkin_LIBRARIES}
                                     ${OpenCV_LIBRARIES}
)

add_executable(stereo_view src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
                                  ${GTK_LIBRARIES}
                                  ${GTK2_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)

install(TARGETS disparity_view image_view_opengl_exe stereo_view
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
