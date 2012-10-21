#The dependencies are libQGLViewer, OpenMesh
#If they are not install into the default directories,
#they should be added here

LIBS    += -L/usr/local/lib/OpenMesh \
        -lOpenMeshCore \
        -lOpenMeshTools
