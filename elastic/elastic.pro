#-------------------------------------------------
#
# Project created by QtCreator 2013-01-19T14:51:09
#
#-------------------------------------------------

QT += core gui opengl xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = elastic
TEMPLATE = lib

CONFIG  += plugin dll

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += ipopt eigen3
}

include (../common.pri)
SOURCES += elastic.cpp \
    optimize.cpp \
    optimize_elastic.cpp \
    eigen_hamilton.cpp

HEADERS += elastic.h \
    optimize.h \
    optimize_elastic.h \
    eigen_hamilton.h

FORMS += \
    elasticpanel.ui

INCLUDEPATH += /usr/local/include/eigen3

LIBS += -lPetMesh -lPGL
LIBS += -lspqr -lcholmod -lsuitesparseconfig -lcamd -lbtf -lcolamd -lumfpack -lklu -lamd

DESTDIR = ..

QMAKE_RPATH = .

unix:!symbian {
    target.path = ../lib/plugins
    INSTALLS += target
}

