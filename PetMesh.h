#ifndef PETMESH_H
#define PETMESH_H

#include <limits.h>

#include <GL/glew.h>


#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <QString>

#define NUM_BUFFERS 8

struct PetTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3f Point;
    typedef OpenMesh::Vec3f Normal;
    typedef OpenMesh::Vec4f Color;

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

    virtual void init(bool isCurve = false);

    bool read_mesh(QString filename);
    virtual bool save(QString filename);

    QString name;
    void SetName(QString m_name);

    PetMesh::Scalar SceneRadius;
    PetMesh::Point SceneCenter;

    OpenMesh::EPropHandleT<bool> showEdge, selectedEdge;
    OpenMesh::VPropHandleT<bool> showVertex, selectedVertex;
    OpenMesh::FPropHandleT<bool> showFace, selectedFace;
    bool showFaces;
    bool showEdges;
    bool showVertices;
    bool visible;
    bool smooth;

    bool *drawProperties[5];

    bool isCurve;

    unsigned int bufferObjs[NUM_BUFFERS];

    float* positions;
    float* normals;
    unsigned int* idxFaces;
    float* colorFaces;

    float* colorEdges;
    float* posEdges;

    float* colorVertices;
    float* posVertices;

    virtual void updateVBO();
    virtual void createVBO();
    virtual void render();

    bool VBOcreated;
    int iSizeofidxFaces;

};


#endif // PETMESH_H
