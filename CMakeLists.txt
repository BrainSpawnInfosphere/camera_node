cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV 2.3.1 REQUIRED)
if(OpenCV_FOUND)
	message("======== ${OpenCV_VERSION} Found ========= ")
endif(OpenCV_FOUND)

rosbuild_add_executable(filter src/filter.cpp )
target_link_libraries(filter  ${OpenCV_LIBS})

rosbuild_add_executable(disparity_view src/test/disparity_view.cpp )
target_link_libraries(disparity_view  ${OpenCV_LIBS})

rosbuild_add_executable(opencv_stereo src/test/stereocamera.cpp )
target_link_libraries(opencv_stereo  ${OpenCV_LIBS})

rosbuild_add_executable(opencv_cam src/opencv_cam.cpp )
target_link_libraries(opencv_cam  ${OpenCV_LIBS})

rosbuild_add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate  ${OpenCV_LIBS})


