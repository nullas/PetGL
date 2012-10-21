#include <QString>
#include <QObject>

#include "PetMesh.h"

int PetMesh::identity = 0;


PetMesh::PetMesh(QString m_name)
{
    this->Identity = identity;
    identity ++;
    if (m_name == NULL)
        this->name = QObject::tr("untitled-");
    else
        this->name = m_name;
}

void PetMesh::SetName(QString m_name)
{
    if (m_name.isEmpty()) return;
    this->name = m_name;
    return;
}

PetMesh::~PetMesh()
{
}
