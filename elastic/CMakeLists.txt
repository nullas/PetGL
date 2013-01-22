cmake_minimum_required (VERSION 2.8)
project (Elastic)


#VERSION

#set (PetGL_VERSION_MAJOR 0)
#set (PetGL_VERSION_MINOR 1)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
include (common)


#find Qt4

#set (QT_USE_QTOPENGL TRUE)
#set (QT_USE_QTXML TRUE)
find_package (Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)


#Qt processes

set (Elastic_SOURCES elastic.cpp)
#set (PetMesh_SOURCES PetMesh.cpp PetCurve.cpp)
#set (PGL_SOURCES PetGL.cpp Viewer.cpp)
set (Elastic_HEADERS elastic.h)
#set (PetGL_FORMS PetGL.ui)

QT4_WRAP_CPP (Elastic_HEADERS_MOC ${Elastic_HEADERS})
#QT4_WRAP_UI (PetGL_FORMS_HEADERS ${PetGL_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#add the executable
add_library (elastic SHARED ${Elastic_SOURCES} ${Elastic_HEADERS_MOC})
target_link_libraries (elastic
	${QT_LIBRARIES}
	${PetMesh}
	${PGL})
set (EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set (LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/plugins")