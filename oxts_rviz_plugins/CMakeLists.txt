cmake_minimum_required(VERSION 3.5)
project(oxts_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(oxts_msgs REQUIRED)

find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)

find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS 
    Core 
    Gui 
    Widgets 
    Test 
    Concurrent
)

set(oxts_rviz_plugins_headers_to_moc
    include/oxts_rviz_plugins/status_panel.hpp
    include/oxts_rviz_plugins/status_widget.hpp
    include/oxts_rviz_plugins/NComRxC.h
    include/oxts_rviz_plugins/nav_const.hpp
    include/oxts_rviz_plugins/QNavLabels.hpp
)

foreach(header "${oxts_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(oxts_rviz_plugins_moc_files "${header}")
endforeach()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
    src/status_panel.cpp
    src/status_widget.cpp
    src/lib/NComRxC.c
    ${oxts_rviz_plugins_moc_files}
)

set (dependencies
    rclcpp  
    class_loader
    pluginlib
    Qt5
    rviz2
    rviz_common
    rviz_default_plugins
    rviz_rendering
    rviz_ogre_vendor
    sensor_msgs
    std_msgs
    oxts_msgs
)

ament_target_dependencies(${library_name} 
    ${dependencies}
)

target_include_directories(${library_name} PUBLIC
    ${Qt5Widgets_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(${library_name} 
    rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
    TARGETS ${library_name}
    EXPORT ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Qt5
    rviz_common
    sensor_msgs
    std_msgs
    rclcpp
)

ament_package()