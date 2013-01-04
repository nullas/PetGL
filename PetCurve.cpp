#include <fstream>

#include <QFileInfo>
#include <QFile>
#include <QString>
#include <QDir>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "PetCurve.h"

#define BUFFER_OFFSET(offset) static_cast<const GLvoid*>(offset)

PetCurve::PetCurve()
{
    this->add_property(isCurveEdge);
    this->add_property(isCurveHalfEdge);
}


bool PetCurve::read_curve(QString filename)
{
    ifstream fin;
    fin.open(filename.toAscii(), ios::in);
    if (!fin.is_open())
    {
        cout<< "Read file error" << filename.toStdString() <<endl;
        return false;
    }
    int begin_vertex, end_vertex;
    QFileInfo fi(filename);
    if (fi.suffix() != "crv") return false;
    QDir fdir = fi.dir();
    QString f_mesh;
    f_mesh = fdir.absoluteFilePath(fi.baseName() + ".ply");
    if (!OpenMesh::IO::read_mesh(*this, f_mesh.toStdString())) return false;
    this->SetName(fi.fileName());
    PetMesh::init(true);

    PetMesh::HalfedgeIter he_it;
    PetMesh::VertexHandle he_begin,he_end;
    PetMesh::EdgeHandle e_hnd;
    PetMesh::EdgeIter e_it;
    for (he_it = this->halfedges_begin(); he_it != this->halfedges_end(); ++he_it)
    {
        this->property(isCurveHalfEdge, *he_it) = true;
    }
    for (e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
    {
        this->property(isCurveEdge, *e_it) = true;
    }
    unsigned int deletededges = 0;
    fin >> begin_vertex >> end_vertex;
    while (!fin.eof())
    {
        for (he_it = this->halfedges_begin(); he_it != this->halfedges_end(); ++he_it)
        {
            he_begin = this->from_vertex_handle(*he_it);
            if (he_begin.idx() == begin_vertex)
            {
                he_end = this->to_vertex_handle(*he_it);
                if ( he_end.idx() == end_vertex)
                {
                    e_hnd = this->edge_handle(*he_it);
                    this->property(isCurveEdge, e_hnd) = false;
                    this->property(isCurveHalfEdge, *he_it) = false;
                    this->property(isCurveHalfEdge, this->opposite_halfedge_handle(*he_it)) = false;
                    this->property(showEdge, e_hnd) = false;
                }//endif ( he_end.idx() == end_vertex)
            }//endif (he_begin.idx() == begin_vertex)
        }//end for
        deletededges++;
        fin >> begin_vertex >> end_vertex;
    }//end while
    fin.close();
    _n_curve_edges = n_edges() - deletededges;
    return true;
}

void PetCurve::init()
{

}


bool PetCurve::save(QString filename)
{
    QFileInfo fi;
    fi.setFile(filename);
    QDir qdir = fi.dir();

    if (fi.suffix() != "crv") return false;
    QString meshname;
    meshname = qdir.absoluteFilePath(fi.baseName() + ".ply");
    if (!PetMesh::save(meshname)) return false;
    ofstream fout(filename.toAscii());
    PetMesh::HalfedgeHandle h_hnd;
    PetMesh::VertexHandle v_hnd;
    if (!fout.is_open())
    {
        cout << "can't open" << filename.toStdString() << endl;
        return false;
    }
    for (PetMesh::EdgeIter e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
    {
        if (!this->property(isCurveEdge, *e_it))
        {
            h_hnd = this->halfedge_handle(*e_it, 0);
            v_hnd = this->from_vertex_handle(h_hnd);
            fout << v_hnd.idx() << " ";
            v_hnd = this->to_vertex_handle(h_hnd);
            fout << v_hnd.idx() << endl;
        }
    }
    fout.close();
    return true;
}

void PetCurve::createVBO()
{
    glGenBuffers(NUM_BUFFERS, bufferObjs);

    colorEdges = new float[8 * sizeof(float) * n_curve_edges()];
    posEdges = new float[6 * sizeof(float) * n_curve_edges()];
    PetMesh::HalfedgeHandle h_hnd;
    PetMesh::VertexHandle v_hnd;
    PetMesh::EdgeIter e_it;
    PetMesh::Point pos;
    unsigned int tmpidx = 0;
    PetMesh::Color tmpcolor;
    for (e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
    {
        if (this->property(isCurveEdge, *e_it))
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
    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float) * n_curve_edges(), colorEdges, GL_DYNAMIC_DRAW);
    delete [] colorEdges;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float) * n_curve_edges(), posEdges, GL_DYNAMIC_DRAW);
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

    VBOcreated = true;

}

void PetCurve::updateVBO()
{
    colorEdges = new float[8 * sizeof(float) * n_curve_edges()];
    posEdges = new float[6 * sizeof(float) * n_curve_edges()];
    PetMesh::HalfedgeHandle h_hnd;
    PetMesh::VertexHandle v_hnd;
    unsigned int tmpidx = 0;
    PetMesh::Color tmpcolor;
    PetMesh::EdgeIter e_it;
    PetMesh::Point pos;
    for (e_it = this->edges_begin(); e_it != this->edges_end(); ++e_it)
    {
        if (this->property(isCurveEdge, *e_it))
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
    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float) * n_curve_edges(), colorEdges, GL_DYNAMIC_DRAW);
    delete [] colorEdges;
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float) * n_curve_edges(), posEdges, GL_DYNAMIC_DRAW);
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

void PetCurve::render()
{
    if (!VBOcreated)
        createVBO();


    if (showEdges)
    {
        glDisable(GL_LIGHTING);
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[4]);
        glColorPointer(4, GL_FLOAT, 0, BUFFER_OFFSET(0));
        glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
        glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
        glLineWidth(2.0);
        glDrawArrays(GL_LINES, 0, 2 * n_curve_edges());
    }
    if (showVertices)
        {
            glPushAttrib(GL_LIGHTING_BIT);
            glDisable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[6]);
            glColorPointer(4, GL_FLOAT, 0, BUFFER_OFFSET(0));
            glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[7]);
            glVertexPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(0));
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, n_vertices());
            glPopAttrib();
    }


}

