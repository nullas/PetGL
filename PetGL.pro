#-------------------------------------------------
#
# Project created by QtCreator 2012-10-15T20:58:57
#
#-------------------------------------------------

QT       += core gui opengl xml
CONFIG  += qt

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PetGL
TEMPLATE = app

DESTDIR = ./q_bin

include (packages.pri)

SOURCES += main.cpp\
    Viewer.cpp \
    PetGL.cpp \
    PetMesh.cpp \
    PetCurve.cpp

HEADERS  += \
    PetGLConfig.h \
    PetMesh.h \
    Viewer.h \
    PetGL.h \
    QStreamRedirect.h \
    PetCurve.h

FORMS    += \
    PetGL.ui

LIBS    += -lQGLViewer
QMAKE_LIBS_OPENGL *= -lGLU


OTHER_FILES += \
    packages.pri
