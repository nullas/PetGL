#The dependencies are libQGLViewer, OpenMesh
#they should be added here

LIBS    += -L/usr/local/lib/OpenMesh \
        -lOpenMeshCore \
        -lOpenMeshTools 
LIBS    += -lQGLViewer
