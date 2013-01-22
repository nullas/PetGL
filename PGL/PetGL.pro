#-------------------------------------------------
#
# Project created by QtCreator 2012-10-15T20:58:57
#
#-------------------------------------------------

QT += core xml opengl gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

include (../externallib.pri)
TARGET = PetGL
TEMPLATE = app
CONFIG += dll

DESTDIR = ..


SOURCES += main.cpp
HEADERS  += PetGL.h

LIBS += -L. -lPGL
LIBS += -L. -lPetMesh


 
