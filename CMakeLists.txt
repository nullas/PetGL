cmake_minimum_required (VERSION 2.8)
project (PetGL)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package (Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)
if (NOT QT4_FOUND)
  message (FATAL_ERROR "Qt4 not found")
endif ()

#find QGLViewer
find_package (QGLViewer)
if (NOT QGLVIEWER_FOUND)
  message (FATAL_ERROR "QGLViewer not found")
endif ()

#find OpenGL
find_package (OpenGL)
if (NOT OPENGL_FOUND)
  message (FATAL_ERROR "OpenGL not found")
endif ()

#find GLEW
find_package (GLEW)
if (NOT GLEW_FOUND)
  message (FATAL_ERROR "GLEW not found")
endif ()

#find OpenMesh
find_package (OpenMesh)
if (NOT OPENMESH_FOUND)
  message (FATAL_ERROR "OpenMesh not found")
endif ()

add_subdirectory (PGL)
include (plugins)

