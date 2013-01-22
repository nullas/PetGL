cmake_minimum_required (VERSION 2.8)
project (PGL)


#VERSION

#set (PetGL_VERSION_MAJOR 0)
#set (PetGL_VERSION_MINOR 1)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")


#find Qt4

#set (QT_USE_QTOPENGL TRUE)
#set (QT_USE_QTXML TRUE)
find_package (Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})


#find QGLViewer
find_package (QGLViewer)
include_directories (${QGLVIEWER_INCLUDE_DIR})


#find OpenGL
find_package (OpenGL)
include_directories (OPENGL_INCLUDE_DIR)

#find GLEW
find_package (GLEW)
include_directories (GLEW_INCLUDE_DIRS)

#find OpenMesh
find_package (OpenMesh)
include_directories (OPENMESH_INCLUDE_DIR)


#configure file
#configure_file (
#	       "${PROJECT_SOURCE_DIR}/PetGLConfig.h.in"
#	       "${PROJECT_BINARY_DIR}/PetGLConfig.h"
#)

#include_directories ("${PROJECT_BINARY_DIR}")


#Qt processes

set (PetGL_SOURCES main.cpp)
set (PetMesh_SOURCES PetMesh.cpp PetCurve.cpp)
set (PGL_SOURCES PetGL.cpp Viewer.cpp )
set (PetGL_HEADERS PetGL.h Viewer.h)
set (PetGL_FORMS PetGL.ui)

QT4_WRAP_CPP (PetGL_HEADERS_MOC ${PetGL_HEADERS})
QT4_WRAP_UI (PetGL_FORMS_HEADERS ${PetGL_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#add the executable
add_library (PetMesh SHARED ${PetMesh_SOURCES})
add_library (PGL SHARED ${PGL_SOURCES} ${PetGL_HEADERS_MOC} ${PetGL_FORMS_HEADERS})
add_executable (PetGL ${PetGL_SOURCES})
target_link_libraries (PetGL ${PETSC_LIBRARIES} 
	${QT_LIBRARIES} 
	${QGLVIEWER_LIBRARIES} 
	${OPENGL_LIBRARIES}
	${OPENMESH_LIBRARIES}
	${GLEW_LIBRARIES}
	PetMesh
	PGL)
set (EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set (LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")