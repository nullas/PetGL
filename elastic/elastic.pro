#-------------------------------------------------
#
# Project created by QtCreator 2013-01-19T14:51:09
#
#-------------------------------------------------

QT += core gui opengl xml

TARGET = elastic
TEMPLATE = lib

CONFIG  += plugin dll

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += ipopt
}

include (../common.pri)
SOURCES += elastic.cpp \
    optimize.cpp

HEADERS += elastic.h \
    optimize.h

LIBS += -lPetMesh -lPGL

DESTDIR = ..

unix:!symbian {
    maemo5 {
        target.path = ../lib/plugins
    } else {
        target.path = ../lib/plugins
    }
    INSTALLS += target
}

FORMS += \
    elasticpanel.ui
