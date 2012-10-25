cmake_minimum_required (VERSION 2.6)
project (PetGL)


#VERSION

set (PetGL_VERSION_MAJOR 0)
set (PetGL_VERSION_MINOR 1)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Module")


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


#find OpenMesh
find_package (OpenMesh)
include_directories (OPENMESH_INCLUDE_DIR)
message (STATUS "cc" ${OPENMESH_LIBRARIES})


#configure file
configure_file (
	       "${PROJECT_SOURCE_DIR}/PetGLConfig.h.in"
	       "${PROJECT_BINARY_DIR}/PetGLConfig.h"
)

include_directories ("${PROJECT_BINARY_DIR}")


#Qt processes

set (PetGL_SOURCES main.cpp PetGL.cpp PetMesh.cpp Viewer.cpp PetCurve.cpp)
set (PetGL_HEADERS PetGL.h Viewer.h)
set (PetGL_FORMS PetGL.ui)

QT4_WRAP_CPP (PetGL_HEADERS_MOC ${PetGL_HEADERS})
QT4_WRAP_UI (PetGL_FORMS_HEADERS ${PetGL_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

#add the executable

add_executable (PetGL ${PetGL_SOURCES} ${PetGL_HEADERS_MOC} ${PetGL_FORMS_HEADERS})
target_link_libraries (PetGL ${PETSC_LIBRARIES} 
	${QT_LIBRARIES} 
	${QGLVIEWER_LIBRARIES} 
	${OPENGL_LIBRARIES}
	${OPENMESH_LIBRARIES})
