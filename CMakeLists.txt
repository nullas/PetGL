cmake_minimum_required (VERSION 2.6)
project (PetGL)

set (PetGL_VERSION_MAJOR 0)
set (PetGL_VERSION_MINOR 1)

configure_file (
	       "${PROJECT_SOURCE_DIR}/PetGLConfig.h.in"
	       "${PROJECT_BINARY_DIR}/PetGLConfig.h"
)

include_directories ("${PROJECt_BINARY_DIR}")

#add the executable
add_executable (PetGL PetGL.cpp)