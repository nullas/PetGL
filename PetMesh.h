#ifndef PETMESH_H
#define PETMESH_H

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <QString>


typedef OpenMesh::PolyMesh_ArrayKernelT<>  PetMesh_T;


class PetMesh : public PetMesh_T
{

public:
    PetMesh(QString m_name = NULL);
    ~PetMesh();


    QString name;
    int Identity;

    static int identity;


    void SetName(QString m_name);
};


#endif // PETMESH_H
