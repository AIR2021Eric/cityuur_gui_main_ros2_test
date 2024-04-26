cmake_minimum_required(VERSION 3.5)
project(cityuur_gui)

# Compile as C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(curos_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(cityuur_gui_SRCS
  src/cityuur_gui/my_plugin.cpp
  src/cityuur_gui/ratio_layouted_frame.cpp
)

set(cityuur_gui_HDRS
  include/cityuur_gui/my_plugin.h
  include/cityuur_gui/ratio_layouted_frame.h
)

set(cityuur_gui_UIS
  resource/my_plugin.ui
)

set(cityuur_gui_INCLUDE_DIRECTORIES
  include
  "${CMAKE_CURRENT_BINARY_DIR}"
)

qt5_wrap_cpp(cityuur_gui_MOCS ${cityuur_gui_HDRS})
qt5_wrap_ui(cityuur_gui_UIS_H ${cityuur_gui_UIS})

include_directories(
  include
  ${cityuur_gui_INCLUDE_DIRECTORIES}
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${cityuur_gui_SRCS}
  ${cityuur_gui_MOCS}
  ${cityuur_gui_UIS_H}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
