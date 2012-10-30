#ifndef PETMESH_H
#define PETMESH_H

#include <limits.h>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <QString>

struct PetTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3f Point;
    typedef OpenMesh::Vec3f Normal;
    typedef OpenMesh::Vec3f Color;

    VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
    FaceAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
    EdgeAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Color);

};

typedef OpenMesh::PolyMesh_ArrayKernelT<PetTraits>  PetMesh_T;


class PetMesh : public PetMesh_T
{

public:
    PetMesh(QString m_name = NULL);
    ~PetMesh();

    void init(bool isCurve = false);

    bool read_mesh(QString filename);
    virtual bool save(QString filename);

    QString name;
    void SetName(QString m_name);

    PetMesh::Scalar SceneRadius;
    PetMesh::Point SceneCenter;

    OpenMesh::EPropHandleT<bool> showEdge;
    OpenMesh::VPropHandleT<bool> showVertex;
    OpenMesh::FPropHandleT<bool> showFace;
    bool showFaces;
    bool showEdges;
    bool showVertices;
    bool visible;
    bool smooth;

    bool *drawProperties[5];

    bool isCurve;
};


#endif // PETMESH_H
