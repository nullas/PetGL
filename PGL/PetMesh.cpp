#include <fstream>

#include <QString>
#include <QObject>
#include <QFileInfo>
#include <QDir>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "PetMesh.h"

using namespace std;

const PetMesh::Color PetMesh::SelectEdgeColor(1,1,1,1);
const PetMesh::Color PetMesh::SelectVertexColor(1,0,0,1);
const PetMesh::Color PetMesh::SelectFaceColor(1,1,1,1);
const PetMesh::Color PetMesh::EdgeColor(0.6, 0.6, 0.6, 1.);
const PetMesh::Color PetMesh::VertexColor(0.8, 0.8, 0.7, 1.);
const PetMesh::Color PetMesh::FaceColor(0.2, 0.3, 0.7, 1.);

PetMesh::PetMesh(QString m_name)
{
    if (m_name == NULL)
        this->name = QObject::tr("untitled-");
    else
        this->name = m_name;

    this->add_property(showEdge);
    this->add_property(showFace);
    this->add_property(showVertex);
    this->add_property(selectedEdge);
    this->add_property(selectedVertex);
    this->add_property(selectedFace);
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


    PetMesh::FaceIter f_it,f_end(faces_end());
    PetMesh::EdgeIter e_it, e_end(edges_end());
    PetMesh::VertexIter v_it,v_end(vertices_end());
    PetMesh::FaceVertexIter fv_it;
    PetMesh::Color f_color(FaceColor);
    PetMesh::Color e_color(EdgeColor);
    PetMesh::Color v_color(VertexColor);

    PetMesh::Point total(0, 0, 0);
    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
    {
        this->set_color(v_it, v_color);
        this->property(showVertex, *v_it) = true;
        this->property(selectedVertex, *v_it) = false;
        total += this->point(v_it);
    }
    SceneCenter = total / this->n_vertices();

    SceneRadius = 0;
    double tempRadius;
    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
    {
        tempRadius = (this->point(v_it)-SceneCenter).norm();
        if (tempRadius > SceneRadius)
                SceneRadius = tempRadius;
    }

    iSizeofidxFaces = 0;
    if (!isCurve)
        {
        for (f_it = this->faces_begin(); f_it != f_end; ++f_it)
        {
            this->set_color(f_it, f_color);
            this->property(showFace, *f_it) = true;
            this->property(selectedFace, *f_it) = false;
            for (fv_it = this->fv_iter(*f_it); fv_it; ++fv_it)
            {
                iSizeofidxFaces++;
            }
        }

    }

    for (e_it = this->edges_begin(); e_it != e_end; ++e_it)
    {
        this->set_color(e_it, e_color);
        this->property(showEdge, *e_it) = true;
        this->property(selectedEdge, *e_it) = false;
    }

    VBOcreated = false;
}


void PetMesh::SetName(QString m_name)
{
    if (m_name.isEmpty()) return;
    this->name = m_name;
    return;
}

PetMesh::~PetMesh()
{
    glDeleteBuffers(NUM_BUFFERS, bufferObjs);
}

bool PetMesh::read_mesh(QString fileName)
{
    if (!OpenMesh::IO::read_mesh(*this, fileName.toStdString())) return false;
    QFileInfo fi(fileName);
    this->SetName(fi.fileName());
    this->init();
    return true;
}

bool PetMesh::save(QString filename)
{
    return OpenMesh::IO::write_mesh(*this, filename.toStdString());
}

void PetMesh::createVBO()
{
    glGenBuffers(NUM_BUFFERS, bufferObjs);

//    PetMesh::FaceIter f_it, f_end(this->faces_end());
//    PetMesh::FaceVertexIter fv_it;
//    PetMesh::Point pos,normal;
//    PetMesh::Color tmpcolor;
//    positions = new float[3 * sizeof(float) * iSizeofidxFaces];
//    normals = new float[3 * sizeof(float) * iSizeofidxFaces];
//    idxFaces = new unsigned int[iSizeofidxFaces + n_faces()];
//    colorFaces = new float[4 * sizeof(float) * iSizeofidxFaces];

//    unsigned int tmpidxface = 0,tmpidx = 0;
//    for (f_it = this->faces_begin(); f_it != f_end; ++f_it)
//    {
//        tmpcolor = this->color(f_it);
//        normal = this->normal(f_it);
//        for (fv_it = this->fv_iter(f_it); fv_it; ++fv_it)
//        {
//            idxFaces[tmpidxface] = tmpidx;
//            pos = this->point(fv_it);
//            normals[3 * tmpidx] = normal[0];
//            normals[3 * tmpidx + 1] = normal[1];
//            normals[3 * tmpidx + 2] = normal[2];
//            colorFaces[4 * tmpidx] = tmpcolor[0];
//            colorFaces[4 * tmpidx + 1] = tmpcolor[1];
//            colorFaces[4 * tmpidx + 2] = tmpcolor[2];
//            colorFaces[4 * tmpidx + 3] = tmpcolor[3];
//            positions[3 * tmpidx] = pos[0];
//            positions[3 * tmpidx + 1] = pos[1];
//            positions[3 * tmpidx + 2] = pos[2];

//            tmpidxface++;
//            tmpidx++;
//        }
//        idxFaces[tmpidxface] = UINT_MAX;
//        tmpidxface++;
//    }

//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[0]);
//    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * iSizeofidxFaces, positions, GL_DYNAMIC_DRAW);
//    delete [] positions;
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[1]);
//    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * iSizeofidxFaces, normals, GL_DYNAMIC_DRAW);
//    delete [] normals;
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferObjs[2]);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (iSizeofidxFaces + n_faces()) * sizeof(unsigned int), idxFaces, GL_STATIC_DRAW);
//    delete [] idxFaces;
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[3]);
//    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * iSizeofidxFaces, colorFaces, GL_DYNAMIC_DRAW);
//    delete [] colorFaces;

//    colorEdges = new float[8 * sizeof(float) * n_edges()];
//    posEdges = new float[6 * sizeof(float) * n_edges()];
//    PetMesh::HalfedgeHandle h_hnd;
//    PetMesh::VertexHandle v_hnd;
//    PetMesh::EdgeIter e_it, e_end(this->edges_end());
//    tmpidx = 0;
//    for (e_it = this->edges_begin(); e_it != e_end; ++e_it)
//    {
//        h_hnd = this->halfedge_handle(*e_it, 0);
//        tmpcolor = this->color(*e_it);
//        v_hnd = this->from_vertex_handle(h_hnd);
//        pos = this->point(v_hnd);
//        posEdges[3 * tmpidx] = pos[0];
//        posEdges[3 * tmpidx + 1] = pos[1];
//        posEdges[3 * tmpidx + 2] = pos[2];
//        colorEdges[4 * tmpidx] = tmpcolor[0];
//        colorEdges[4 * tmpidx + 1] = tmpcolor[1];
//        colorEdges[4 * tmpidx + 2] = tmpcolor[2];
//        colorEdges[4 * tmpidx + 3] = tmpcolor[3];
//        tmpidx++;
//        v_hnd = this->to_vertex_handle(h_hnd);
//        pos = this->point(v_hnd);
//        posEdges[3 * tmpidx] = pos[0];
//        posEdges[3 * tmpidx + 1] = pos[1];
//        posEdges[3 * tmpidx + 2] = pos[2];
//        colorEdges[4 * tmpidx] = tmpcolor[0];
//        colorEdges[4 * tmpidx + 1] = tmpcolor[1];
//        colorEdges[4 * tmpidx + 2] = tmpcolor[2];
//        colorEdges[4 * tmpidx + 3] = tmpcolor[3];
//        tmpidx++;

//    }
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
//    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float) * n_edges(), colorEdges, GL_DYNAMIC_DRAW);
//    delete [] colorEdges;
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
//    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float) * n_edges(), posEdges, GL_DYNAMIC_DRAW);
//    delete [] posEdges;

//    colorVertices = new float[4 * sizeof(float) * n_vertices()];
//    posVertices = new float[3 * sizeof(float) * n_vertices()];
//    PetMesh::VertexIter v_it, v_end(this->vertices_end());
//    tmpidx = 0;
//    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
//    {
//        pos = this->point(*v_it);
//        tmpcolor = this->color(*v_it);
//        colorVertices[tmpidx * 4] = tmpcolor[0];
//        colorVertices[tmpidx * 4+ 1] = tmpcolor[1];
//        colorVertices[tmpidx * 4+ 2] = tmpcolor[2];
//        colorVertices[tmpidx * 4+ 3] = tmpcolor[3];
//        posVertices[tmpidx * 3] = pos[0];
//        posVertices[tmpidx * 3 + 1] = pos[1];
//        posVertices[tmpidx * 3 + 2] = pos[2];
//        ++tmpidx;
//    }
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[6]);
//    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * n_vertices(), colorVertices, GL_DYNAMIC_DRAW);
//    delete [] colorVertices;
//    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[7]);
//    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * n_vertices(), posVertices, GL_DYNAMIC_DRAW);
//    delete [] posVertices;
    updateVBO();
    VBOcreated = true;

}

void PetMesh::updateVBO()
{
    update_normals();
    PetMesh::FaceIter f_it, f_end(this->faces_end());
    PetMesh::FaceVertexIter fv_it;
    PetMesh::Point pos,normal;
    PetMesh::Color tmpcolor;
    positions = new float[3 * sizeof(float) * iSizeofidxFaces];
    normals = new float[3 * sizeof(float) * iSizeofidxFaces];
    idxFaces = new unsigned int[iSizeofidxFaces + n_faces()];
    colorFaces = new float[4 * sizeof(float) * iSizeofidxFaces];

    unsigned int tmpidxface = 0,tmpidx = 0;
    for (f_it = this->faces_begin(); f_it != f_end; ++f_it)
    {
        tmpcolor = this->color(f_it);
        normal = this->normal(f_it);
        for (fv_it = this->fv_iter(f_it); fv_it; ++fv_it)
        {
            idxFaces[tmpidxface] = tmpidx;
            pos = this->point(fv_it);
            normals[3 * tmpidx] = normal[0];
            normals[3 * tmpidx + 1] = normal[1];
            normals[3 * tmpidx + 2] = normal[2];
            colorFaces[4 * tmpidx] = tmpcolor[0];
            colorFaces[4 * tmpidx + 1] = tmpcolor[1];
            colorFaces[4 * tmpidx + 2] = tmpcolor[2];
            colorFaces[4 * tmpidx + 3] = tmpcolor[3];
            positions[3 * tmpidx] = pos[0];
            positions[3 * tmpidx + 1] = pos[1];
            positions[3 * tmpidx + 2] = pos[2];

            tmpidxface++;
            tmpidx++;
        }
        idxFaces[tmpidxface] = UINT_MAX;
        tmpidxface++;
    }

    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[0]);
    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * iSizeofidxFaces, positions, GL_DYNAMIC_DRAW);
    delete [] positions;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[1]);
    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * iSizeofidxFaces, normals, GL_DYNAMIC_DRAW);
    delete [] normals;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferObjs[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (iSizeofidxFaces + n_faces()) * sizeof(unsigned int), idxFaces, GL_STATIC_DRAW);
    delete [] idxFaces;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[3]);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * iSizeofidxFaces, colorFaces, GL_DYNAMIC_DRAW);
    delete [] colorFaces;

    colorEdges = new float[8 * sizeof(float) * n_edges()];
    posEdges = new float[6 * sizeof(float) * n_edges()];
    PetMesh::HalfedgeHandle h_hnd;
    PetMesh::VertexHandle v_hnd;
    PetMesh::EdgeIter e_it, e_end(this->edges_end());
    tmpidx = 0;
    for (e_it = this->edges_begin(); e_it != e_end; ++e_it)
    {
        h_hnd = this->halfedge_handle(*e_it, 0);
        tmpcolor = this->color(*e_it);
        v_hnd = this->from_vertex_handle(h_hnd);
        pos = this->point(v_hnd);
        posEdges[3 * tmpidx] = pos[0];
        posEdges[3 * tmpidx + 1] = pos[1];
        posEdges[3 * tmpidx + 2] = pos[2];
        colorEdges[4 * tmpidx] = tmpcolor[0];
        colorEdges[4 * tmpidx + 1] = tmpcolor[1];
        colorEdges[4 * tmpidx + 2] = tmpcolor[2];
        colorEdges[4 * tmpidx + 3] = tmpcolor[3];
        tmpidx++;
        v_hnd = this->to_vertex_handle(h_hnd);
        pos = this->point(v_hnd);
        posEdges[3 * tmpidx] = pos[0];
        posEdges[3 * tmpidx + 1] = pos[1];
        posEdges[3 * tmpidx + 2] = pos[2];
        colorEdges[4 * tmpidx] = tmpcolor[0];
        colorEdges[4 * tmpidx + 1] = tmpcolor[1];
        colorEdges[4 * tmpidx + 2] = tmpcolor[2];
        colorEdges[4 * tmpidx + 3] = tmpcolor[3];
        tmpidx++;

    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float) * n_edges(), colorEdges, GL_DYNAMIC_DRAW);
    delete [] colorEdges;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float) * n_edges(), posEdges, GL_DYNAMIC_DRAW);
    delete [] posEdges;

    colorVertices = new float[4 * sizeof(float) * n_vertices()];
    posVertices = new float[3 * sizeof(float) * n_vertices()];
    PetMesh::VertexIter v_it, v_end(this->vertices_end());
    tmpidx = 0;
    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
    {
        pos = this->point(*v_it);
        tmpcolor = this->color(*v_it);
        colorVertices[tmpidx * 4] = tmpcolor[0];
        colorVertices[tmpidx * 4+ 1] = tmpcolor[1];
        colorVertices[tmpidx * 4+ 2] = tmpcolor[2];
        colorVertices[tmpidx * 4+ 3] = tmpcolor[3];
        posVertices[tmpidx * 3] = pos[0];
        posVertices[tmpidx * 3 + 1] = pos[1];
        posVertices[tmpidx * 3 + 2] = pos[2];
        ++tmpidx;
    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[6]);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * n_vertices(), colorVertices, GL_DYNAMIC_DRAW);
    delete [] colorVertices;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[7]);
    glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * n_vertices(), posVertices, GL_DYNAMIC_DRAW);
    delete [] posVertices;
}

void PetMesh::render()
{
    if (!VBOcreated)
        createVBO();

    if (showFaces)
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glEnable(GL_LIGHTING);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[0]);
        glVertexPointer(3, GL_FLOAT, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[1]);
        glNormalPointer(GL_FLOAT, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[3]);
        glColorPointer(4, GL_FLOAT, 0, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferObjs[2]);
        glDrawElements(GL_POLYGON, iSizeofidxFaces + n_faces(), GL_UNSIGNED_INT, 0);
        glPopAttrib();
    }

    if (showEdges)
    {
        glPushAttrib(GL_LIGHTING_BIT);
        glDisable(GL_LIGHTING);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
        glColorPointer(4, GL_FLOAT, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
        glVertexPointer(3, GL_FLOAT, 0, 0);
        glLineWidth(2.0);
        glDrawArrays(GL_LINES, 0, 2 * n_edges());
        glPopAttrib();
    }
    if (showVertices)
        {
            glPushAttrib(GL_LIGHTING_BIT);
            glDisable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[6]);
            glColorPointer(4, GL_FLOAT, 0, 0);
            glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[7]);
            glVertexPointer(3, GL_FLOAT, 0, 0);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, n_vertices());
            glPopAttrib();
    }
}
