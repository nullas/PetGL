#ifndef PLUGININTERFACE_H
#define PLUGININTERFACE_H

#include <QObject>
#include <QtPlugin>

#include "PluginInterface.h"


class PetPluginInterface : public QObject
{
    Q_OBJECT

signals:
    void updateViewNeeded();
public slots:

public:
    virtual ~PetPluginInterface() {}
    virtual void initial(QWidget* parent) = 0;
    virtual void drawExtra() = 0;
};

Q_DECLARE_INTERFACE (PetPluginInterface, "plugins/1.0")



#endif // PLUGININTERFACE_H 
