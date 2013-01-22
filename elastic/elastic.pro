#-------------------------------------------------
#
# Project created by QtCreator 2013-01-19T14:51:09
#
#-------------------------------------------------

QT += core gui opengl xml

TARGET = elastic
TEMPLATE = lib

CONFIG  += plugin dll

include (../common.pri)
SOURCES += elastic.cpp

HEADERS += elastic.h

LIBS += -lPetMesh -lPGL

DESTDIR = ../plugins

unix:!symbian {
    maemo5 {
        target.path = ../lib/plugins
    } else {
        target.path = ../lib/plugins
    }
    INSTALLS += target
}
