TEMPLATE = subdirs
#must be built

SUBDIRS = PGL1 PGL2 PGL3

PGL1.file = PGL/PetMesh.pro
PGL2.file = PGL/PGL.pro
PGL3.file = PGL/PetGL.pro

#plugins
include (plugins.pri)

CONFIG += ordered
