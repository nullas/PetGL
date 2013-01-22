#ifndef ELASTIC_H
#define ELASTIC_H

#include <QObject>
#include <QtPlugin>

#include "PluginInterface.h"
#include "PetGL.h"


class Elastic : public QObject, PetPluginInterface
{
    Q_OBJECT;
    Q_INTERFACES(PetPluginInterface)
public:
    Elastic();
    void initial(QWidget* parent);
private:
    PetGL* parent;
};

#endif // ELASTIC_H
