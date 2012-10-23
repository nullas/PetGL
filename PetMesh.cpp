#include <QString>
#include <QObject>

#include "PetMesh.h"

using namespace std;

int PetMesh::identity = 0;


PetMesh::PetMesh(QString m_name)
{
    this->Identity = identity;
    identity ++;
    if (m_name == NULL)
        this->name = QObject::tr("untitled-");
    else
        this->name = m_name;

    showFaces = true;
    showEdges = false;
    showVertices = false;
    visible = true;

    smooth = false;

    drawProperties[0] = &visible;
    drawProperties[1] = &showFaces;
    drawProperties[2] = &showEdges;
    drawProperties[3] = &showVertices;
    drawProperties[4] = &smooth;
}

void PetMesh::init()
{

    this->request_face_normals();
    this->request_vertex_normals();
    this->update_normals();

    this->request_face_colors();
    this->request_edge_colors();
    this->request_vertex_colors();

    PetMesh::FaceIter f_it;
    PetMesh::EdgeIter e_it;
    PetMesh::VertexIter v_it;
    PetMesh::Color f_color(0.25,0.4,0.8);
    PetMesh::Color e_color(0.6,0.6,0.6);
    PetMesh::Color v_color(1.,0.,0.);
    for (f_it = this->faces_begin(); f_it != this->faces_end(); ++f_it) this->set_color(f_it, f_color);
    for (e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it) this->set_color(e_it, e_color);
    for (v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it) this->set_color(v_it, v_color);
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
