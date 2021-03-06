# - Try to findOPENMESH
# Once done this will define
#  
# OPENMESH_FOUND           - system has OPENMESH
# OPENMESH_INCLUDE_DIR - theOPENMESH include directory
# OPENMESH_LIBRARY_DIR  - Library DIR of OPENMESH
# OPENMESH_LIBRARIES      - Libraries of OPENMESH

IF (OPENMESH_INCLUDE_DIR)
  # Already in cache, be silent
  SET(OPENMESH_FIND_QUIETLY TRUE)
ENDIF (OPENMESH_INCLUDE_DIR)

FIND_PATH(OPENMESH_INCLUDE_DIR OpenMesh/Core/Mesh/PolyMeshT.hh
	  PATHS /usr/local/include 
                /usr/include 
                /usr/local/OpenMesh-2.0rc4/include
		"$ENV{HOME}/usr/include"
                "${CMAKE_SOURCE_DIR}/OpenMesh/src"
                "${CMAKE_SOURCE_DIR}/libs_required/OpenMesh/src"
                "${CMAKE_SOURCE_DIR}/../OpenMesh/src"
                /ACG/acgdev/gcc-4.0-x86_64/OM2/OpenMesh-2.0/installed/include
		    "C:\\Program Files\\OpenMesh 2.0\\include"
                )

find_library(OPENMESH_LIBRARIES_DEBUG
             NAMES OpenMeshToolsd
             PATHS /usr/lib
                   /usr/local/lib
		   "$ENV{HOME}/usr/lib"
                   ENV LD_LIBRARY_PATH
                   ENV LIBRARY_PATH
             PATH_SUFFIXES OpenMesh
            )

find_library(OPENMESH_LIBRARIES_DEBUG_
             NAMES OpenMeshCored 
             PATHS /usr/lib
                   /usr/local/lib
		   "$ENV{HOME}/usr/lib"
                   ENV LD_LIBRARY_PATH
                   ENV LIBRARY_PATH
             PATH_SUFFIXES OpenMesh
            )

find_library(OPENMESH_LIBRARIES_RELEASE
             NAMES OpenMeshTools
             PATHS /usr/lib
                   /usr/local/lib
		   "$ENV{HOME}/usr/lib"
                   ENV LD_LIBRARY_PATH
                   ENV LIBRARY_PATH
             PATH_SUFFIXES OpenMesh
            )
find_library(OPENMESH_LIBRARIES_RELEASE_
             NAMES OpenMeshCore
             PATHS /usr/lib
                   /usr/local/lib
		   "$ENV{HOME}/usr/lib"
                   ENV LD_LIBRARY_PATH
                   ENV LIBRARY_PATH
             PATH_SUFFIXES OpenMesh
            )

if(OPENMESH_LIBRARIES_RELEASE AND OPENMESH_LIBRARIES_RELEASE_)
  if(OPENMESH_LIBRARIES_DEBUG AND OPENMESH_LIBRARIES_DEBUG_)
	set(OPENMESH_LIBRARIES optimized ${OPENMESH_LIBRARIES_RELEASE} ${OPENMESH_LIBRARIES_RELEASE_}
				debug ${OPENMESH_LIBRARIES_DEBUG} ${OPENMESH_LIBRARIES_DEBUG_})
  else()
	  set(OPENMESH_LIBRARIES ${OPENMESH_LIBRARIES_RELEASE} ${OPENMESH_LIBRARIES_RELEASE_})
  endif()
else()
  if(OPENMESH_LIBRARIES_DEBUG AND OPENMESH_LIBRARIES_DEBUG_)
	set(OPENMESH_LIBRARIES ${OPENMESH_LIBRARIES_DEBUG} ${OPENMESH_LIBRARIES_DEBUG_})
  endif()
endif()

IF (OPENMESH_INCLUDE_DIR AND OPENMESH_LIBRARIES)
   SET(OPENMESH_FOUND TRUE)
ENDIF ()

IF (NOT OPENMESH_FOUND)
   SET(OPENMESH_FOUND FALSE )
ENDIF ()

