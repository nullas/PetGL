#-------------------------------------------------
#
# Project created by QtCreator 2012-10-15T20:58:57
#
#-------------------------------------------------

QT += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PGL
TEMPLATE = lib

#DESTDIR = .

include (../externallib.pri)

SOURCES += \
    Viewer.cpp \
    PetGL.cpp

HEADERS  += \
    PetMesh.h \
    Viewer.h \
    PetGL.h \
    QStreamRedirect.h \
    PetCurve.h \
    PluginInterface.h

FORMS += PetGL.ui

LIBS += -L. -lPetMesh

QMAKE_LIBS_OPENGL *= -lGLU -lGLEW


