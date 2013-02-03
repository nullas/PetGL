cmake_minimum_required (VERSION 2.8)
project (Elastic)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
include (common)


#find Qt4

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
set (Elastic_FORMS elasticpanel.ui)

QT4_WRAP_CPP (Elastic_HEADERS_MOC ${Elastic_HEADERS})
QT4_WRAP_UI (Elastic_FORMS_HEADERS ${Elastic_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#add the executable
add_library (elastic SHARED ${Elastic_SOURCES} ${Elastic_HEADERS_MOC} ${Elastic_FORMS_HEADERS})
target_link_libraries (elastic
	${QT_LIBRARIES}
	${PetMesh}
	${PGL})
set (EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set (LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")