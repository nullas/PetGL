#ifndef PETMESH_H
#define PETMESH_H

#include <limits.h>
#include <vector>

#include <GL/glew.h>


#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <QString>

#define NUM_BUFFERS 10
//8 for mesh and 2 temporary

struct PetTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3d Point;
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

    virtual unsigned int v_n_edges() { return n_edges();}

    virtual bool iscurve();

    bool read_mesh(QString filename);
    virtual bool save(QString filename);
    bool dumpToCSV(QString filename);

    QString name;
    void SetName(QString m_name);

    void computeScene();
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
    virtual void drawPickEdges();
    virtual void drawPickVertices();

    bool VBOcreated;
    int num_of_triangles_;
    float PointSize;

    static const PetMesh::Color FaceColor, VertexColor, EdgeColor;
    static const PetMesh::Color SelectFaceColor, SelectVertexColor, SelectEdgeColor;
    void setVerticesSelected(const std::vector<unsigned int>&);
    void setVertexSelected(const unsigned int);
    void setFacesSelected(const std::vector<unsigned int>&);
    void setFaceSelected(const unsigned int);

    void setEdgesSelected(const std::vector<unsigned int>&);
    void setEdgeSelected(const unsigned int);

    void setVerticesUnelected(const std::vector<unsigned int>&);
    void setVertexUnselected(const unsigned int);

    void setFacesUnselected(const std::vector<unsigned int>&);
    void setFaceUnselected(const unsigned int);

    void setEdgesUnselected(const std::vector<unsigned int>&);
    void setEdgeUnselected(const unsigned int);

    void getSelectedVertices(std::vector<unsigned int>&);
    void getSelectedFaces(std::vector<unsigned int>&);
    virtual void getSelectedEdges(std::vector<unsigned int>&);

    //Handle based

    void setVerticesSelected(const std::vector<PetMesh::VertexHandle>&);
    void setVertexSelected(const PetMesh::VertexHandle);

    void setFacesSelected(const std::vector<PetMesh::FaceHandle>&);
    void setFaceSelected(const PetMesh::FaceHandle);

    void setEdgesSelected(const std::vector<PetMesh::EdgeHandle>&);
    void setEdgeSelected(const PetMesh::EdgeHandle);

    void setVerticesUnelected(const std::vector<PetMesh::VertexHandle>&);
    void setVertexUnselected(const PetMesh::VertexHandle);

    void setFacesUnselected(const std::vector<PetMesh::FaceHandle>&);
    void setFaceUnselected(const PetMesh::FaceHandle);

    void setEdgesUnselected(const std::vector<PetMesh::EdgeHandle>&);
    void setEdgeUnselected(const PetMesh::EdgeHandle);

    void getSelectedVertices(std::vector<PetMesh::VertexHandle>&);
    void getSelectedFaces(std::vector<PetMesh::FaceHandle>&);
    virtual void getSelectedEdges(std::vector<PetMesh::EdgeHandle>&);

    void setVerticesSelectedByFace(const PetMesh::FaceHandle& hnd);
    void setVerticesSelectedByFaces(const std::vector<PetMesh::FaceHandle>& hnd);

    void clearSelectedVertices();
    void clearSelectedEdges();
    void clearSelectedFaces();
};


#endif // PETMESH_H
