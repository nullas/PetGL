#include <QString>
#include <QObject>
#include <QFileInfo>

#include <OpenMesh/Core/IO/MeshIO.hh>

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

    this->add_property(showEdge);
    this->add_property(showFace);
    this->add_property(showVertex);
}


void PetMesh::init(bool isCurve)
{
    this->isCurve = isCurve;

    if (isCurve)
    {
        showFaces = false;
        showEdges = true;
    }
    else
    {
        showFaces = true;
        showEdges = false;
        this->update_normals();
    }

    showVertices = false;
    visible = true;

    smooth = false;

    drawProperties[0] = &visible;
    drawProperties[1] = &showFaces;
    drawProperties[2] = &showEdges;
    drawProperties[3] = &showVertices;
    drawProperties[4] = &smooth;

    PetMesh::FaceIter f_it;
    PetMesh::EdgeIter e_it;
    PetMesh::VertexIter v_it;
    PetMesh::Color f_color(0.2, 0.3, 0.7);
    PetMesh::Color e_color(0.6, 0.6, 0.6);
    PetMesh::Color v_color(1., 0., 0.);
    for (f_it = this->faces_begin(); f_it != this->faces_end(); ++f_it)
    {
        this->set_color(f_it, f_color);
        this->property(showFace, *f_it) = true;
    }
    for (e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
    {
        this->set_color(e_it, e_color);
        this->property(showEdge, *e_it) = true;
    }
    for (v_it = this->vertices_begin(); v_it != this->vertices_end(); ++v_it)
    {
        this->set_color(v_it, v_color);
        this->property(showVertex, *v_it) = true;
    }
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

bool PetMesh::read_mesh(QString fileName)
{
    if (!OpenMesh::IO::read_mesh(*this, fileName.toStdString())) return false;
    QFileInfo fi(fileName);
    this->SetName(fi.fileName());
    this->init();
    return true;
}
