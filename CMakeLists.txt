cmake_minimum_required(VERSION 2.8)
project(kmi_image_saver)

find_package(catkin REQUIRED COMPONENTS camera_calibration_parsers cv_bridge image_transport nodelet roscpp std_srvs)

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

# Extra tools
add_executable(image_saver src/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)
