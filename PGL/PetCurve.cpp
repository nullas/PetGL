#include <fstream>
#include <utility>
#include <algorithm>

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


bool PetCurve::iscurve()
{
    return true;
}

bool PetCurve::read_curve(QString filename)
{
    ifstream fin;
    fin.open(filename.toLocal8Bit().data(), ios::in);
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
    fin >> begin_vertex ;
    std::map<int,int> edges;
    while (!fin.eof())
    {
        fin >> end_vertex;
        edges.insert(std::map<int,int>::value_type(begin_vertex, end_vertex));
        fin >> begin_vertex;
    }

    for (he_it = this->halfedges_begin(); he_it != this->halfedges_end(); ++he_it)
    {
        he_begin = this->from_vertex_handle(*he_it);
        if (edges.count(he_begin.idx()))
        {
            he_end = this->to_vertex_handle(*he_it);
            if (he_end.idx() == edges[he_begin.idx()])
            {
                e_hnd = this->edge_handle(*he_it);
                this->property(isCurveEdge, e_hnd) = false;
                this->property(isCurveHalfEdge, *he_it) = false;
                this->property(isCurveHalfEdge, this->opposite_halfedge_handle(*he_it)) = false;
                this->property(showEdge, e_hnd) = false;
                deletededges++;
            }//endif find
        }//endif find
    }//end for

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
    ofstream fout(filename.toLocal8Bit().data());
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

void PetCurve::drawPickEdges()
{
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    unsigned int n_edges = v_n_edges();
    unsigned char* pickColor = new unsigned char[8 * n_edges];
    unsigned char r=0,g=0,b=0;
    long int i = 0, tmpidx;
    PetMesh::EdgeIter e_end(edges_end());
    PetMesh::EdgeIter e_it = edges_begin();
    for (; e_it != e_end; ++e_it)
    {
        if (!property(this->isCurveEdge, e_it.handle())) continue;
        tmpidx = e_it.handle().idx();
        r =  tmpidx % 256;
        tmpidx /= 256;
        g = tmpidx % 256;
        tmpidx /= 256;
        b = tmpidx;
        if (b == 255) return;

        pickColor[i * 4] = r;
        pickColor[i * 4 + 1] = g;
        pickColor[i * 4 + 2] = b;
        pickColor[i * 4 + 3] = 1;
        ++i;
        pickColor[i * 4] = r;
        pickColor[i * 4 + 1] = g;
        pickColor[i * 4 + 2] = b;
        pickColor[i * 4 + 3] = 1;
        ++i;
    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[8]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(unsigned char) * n_edges, pickColor, GL_DYNAMIC_DRAW);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);

    delete [] pickColor;

    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[5]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glLineWidth(2.0);
    glDrawArrays(GL_LINES, 0, 2 * n_edges);
    glPopAttrib();
}

void PetCurve::drawPickVertices()
{

    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    unsigned int n_vertice = n_vertices();
    unsigned char* pickColor = new unsigned char[4 * n_vertice];
    unsigned char r=0,g=0,b=0;
    long int i = 0, tmpidx;
    PetMesh::VertexIter v_end(vertices_end());
    for (PetMesh::VertexIter v_it = vertices_begin(); v_it != v_end; ++v_it)
    {
        tmpidx = v_it.handle().idx();
        r =  tmpidx % 256;
        tmpidx /= 256;
        g = tmpidx % 256;
        tmpidx /= 256;
        b = tmpidx;
        if (b == 255) return;

        pickColor[i * 4] = r;
        pickColor[i * 4 + 1] = g;
        pickColor[i * 4 + 2] = b;
        pickColor[i * 4 + 3] = 1;
        ++i;
    }
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[8]);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(unsigned char) * n_vertice, pickColor, GL_DYNAMIC_DRAW);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);

    delete [] pickColor;

    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[7]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glPointSize(2.0);
    glDrawArrays(GL_POINTS, 0, n_vertice);
    glPopAttrib();
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
            glPointSize(PointSize);
            glDrawArrays(GL_POINTS, 0, n_vertices());
            glPopAttrib();
    }
}

void PetCurve::getSelectedEdges(std::vector<PetMesh::EdgeHandle> & idx)
{
    idx.clear();
    PetMesh::EdgeIter e_it = edges_begin(), e_end = edges_end();
    for (; e_it != e_end; ++e_it)
    {
        if (this->property(selectedEdge, e_it) && this->property(isCurveEdge, e_it))
            idx.push_back(*e_it);
    }
}

void PetCurve::getSelectedEdges(std::vector<unsigned int> & idx)
{
    idx.clear();
    PetMesh::EdgeIter e_it = edges_begin(), e_end = edges_end();
    for (; e_it != e_end; ++e_it)
    {
        if (this->property(selectedEdge, e_it) && this->property(isCurveEdge, e_it))
            idx.push_back((*e_it).idx());
    }
}

void PetCurve::setCurveSelected(const unsigned int idx)
{
    setFaceSelected(idx);
    PetCurve::CurveEdgeIter ce_it = this->fe_iter(this->face_handle(idx));
    for (; ce_it; ++ce_it)
    {
        setEdgeSelected(ce_it.handle().idx());
    }
}

void PetCurve::setCurveUnselected(const unsigned int idx)
{
    setFaceUnselected(idx);
    PetCurve::CurveEdgeIter ce_it = this->fe_iter(this->face_handle(idx));
    for (; ce_it; ++ce_it)
    {
        setEdgeUnselected(ce_it.handle().idx());
    }
}

void PetCurve::setCurvesSelected(const std::vector<unsigned int>& idx)
{
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setCurveSelected(*it);
    }
}

void PetCurve::setCurvesUnselected(const std::vector<unsigned int>& idx)
{
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setCurveSelected(*it);
    }
}

void PetCurve::getSelectedCurves(std::vector<unsigned int>& idx)
{
    getSelectedFaces(idx);
}

void PetCurve::getSelectedCurves(std::vector<PetCurve::CurveHandle>& idx)
{
    getSelectedFaces(idx);
}

void PetCurve::setVerticesSelectedByCurve(const PetCurve::CurveHandle &hnd)
{
    setVerticesSelectedByFace(hnd);
}

void PetCurve::setVerticesSelectedByCurves(const std::vector<PetCurve::CurveHandle> &hnd)
{
    setVerticesSelectedByFaces(hnd);
}

void PetCurve::clearSelectedCurves()
{
    clearSelectedEdges();
    clearSelectedFaces();
}
