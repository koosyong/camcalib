cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## include
set( sources
    src/calibrationOneCam.cpp
    src/mainwindow.cpp
    src/dialogcalibration.cpp
    src/ckinectcalibration.cpp
    src/ckinectthread.cpp
    src/ckinecttfpublisher.cpp
)

set( headers
    src/mainwindow.h
    src/dialogcalibration.h
    src/ckinectcalibration.h
    src/ckinectthread.h
    src/ckinecttfpublisher.h
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
)

## Qt
find_package(Qt4)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
qt4_wrap_cpp(moc_sources ${headers})

## Ros build
rosbuild_add_executable(camcalib ${sources} ${headers}
    ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${moc_sources})
target_link_libraries(camcalib ${OPENGL_LIBRARIES} ${QT_LIBRARIES})
