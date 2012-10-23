#ifndef PETMESH_H
#define PETMESH_H

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <QString>

struct PetTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
    typedef OpenMesh::Vec3d Color;
};

typedef OpenMesh::PolyMesh_ArrayKernelT<PetTraits>  PetMesh_T;


class PetMesh : public PetMesh_T
{

public:
    PetMesh(QString m_name = NULL);
    ~PetMesh();

    void init();


    QString name;
    int Identity;

    static int identity;


    void SetName(QString m_name);

    bool showFaces;
    bool showEdges;
    bool showVertices;
    bool visible;

    bool smooth;

    bool *drawProperties[5];
};


#endif // PETMESH_H
