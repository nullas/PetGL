#-------------------------------------------------
#
# Project created by QtCreator 2012-10-15T20:58:57
#
#-------------------------------------------------

QT += core

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PetMesh
TEMPLATE = lib

DESTDIR = ..

include (../externallib.pri)

SOURCES += PetMesh.cpp \
    PetCurve.cpp

HEADERS  += PetMesh.h \
    PetCurve.h




 
