#include <fstream>

#include <QString>
#include <QObject>
#include <QFileInfo>
#include <QDir>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "PetMesh.h"
#include "defs.h"

using namespace std;

const PetMesh::Color PetMesh::SelectEdgeColor = OpenMesh::color_cast<PetMesh::Color>(OpenMesh::Vec4uc(20,20,20,255));
const PetMesh::Color PetMesh::SelectVertexColor = OpenMesh::color_cast<PetMesh::Color>(OpenMesh::Vec4uc(220,87,18,255));
const PetMesh::Color PetMesh::SelectFaceColor = OpenMesh::color_cast<PetMesh::Color>(OpenMesh::Vec4uc(17,64,108,255));
const PetMesh::Color PetMesh::EdgeColor= OpenMesh::color_cast<PetMesh::Color>(OpenMesh::Vec4uc(36,169,225,255));
const PetMesh::Color PetMesh::VertexColor = OpenMesh::color_cast<PetMesh::Color>(OpenMesh::Vec4uc(138,151,123,255));
const PetMesh::Color PetMesh::FaceColor(0.2, 0.3, 0.7, 1.);

PetMesh::PetMesh(QString m_name)
{
    if (m_name == NULL)
        this->name = QObject::tr("untitled-");
    else
        this->name = m_name;

    PointSize = _PointSize;

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

    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
    {
        this->set_color(v_it, v_color);
        this->property(showVertex, *v_it) = true;
        this->property(selectedVertex, *v_it) = false;
    }
    SceneCenter = PetMesh::Point(0,0,0);

    SceneRadius = 1;

    VBOcreated = false;
}

bool PetMesh::iscurve()
{
    return false;
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
    updateVBO();
    VBOcreated = true;
}

void PetMesh::updateVBO()
{
    update_normals();
    PetMesh::FaceIter f_it, f_end(this->faces_end());
    PetMesh::FaceVertexIter fv_it;
    PetMesh::Point pos;
    PetMesh::Normal normal;
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
            positions[3 * tmpidx] = (float)pos[0];
            positions[3 * tmpidx + 1] = (float)pos[1];
            positions[3 * tmpidx + 2] = (float)pos[2];

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
        posEdges[3 * tmpidx] = (float)pos[0];
        posEdges[3 * tmpidx + 1] = (float)pos[1];
        posEdges[3 * tmpidx + 2] = (float)pos[2];
        colorEdges[4 * tmpidx] = tmpcolor[0];
        colorEdges[4 * tmpidx + 1] = tmpcolor[1];
        colorEdges[4 * tmpidx + 2] = tmpcolor[2];
        colorEdges[4 * tmpidx + 3] = tmpcolor[3];
        tmpidx++;
        v_hnd = this->to_vertex_handle(h_hnd);
        pos = this->point(v_hnd);
        posEdges[3 * tmpidx] = (float)pos[0];
        posEdges[3 * tmpidx + 1] = (float)pos[1];
        posEdges[3 * tmpidx + 2] = (float)pos[2];
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
        posVertices[tmpidx * 3] = (float)pos[0];
        posVertices[tmpidx * 3 + 1] = (float)pos[1];
        posVertices[tmpidx * 3 + 2] = (float)pos[2];
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
        glPointSize(PointSize);
        glDrawArrays(GL_POINTS, 0, n_vertices());
        glPopAttrib();
    }
}

void PetMesh::drawPickVertices()
{

    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1.,1.,1.);
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[0]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferObjs[2]);
    glDrawElements(GL_POLYGON, iSizeofidxFaces + n_faces(), GL_UNSIGNED_INT, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
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

void PetMesh::drawPickEdges()
{
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1.,1.,1.);
    glBindBuffer(GL_ARRAY_BUFFER, bufferObjs[0]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferObjs[2]);
    glDrawElements(GL_POLYGON, iSizeofidxFaces + n_faces(), GL_UNSIGNED_INT, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    unsigned int n_edges = v_n_edges();
    unsigned char* pickColor = new unsigned char[8 * n_edges];
    unsigned char r=0,g=0,b=0;
    long int i = 0, tmpidx;
    PetMesh::EdgeIter e_end(edges_end());
    PetMesh::EdgeIter e_it = edges_begin();
    for (; e_it != e_end; ++e_it)
    {
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
    glLineWidth(3.0);
    glDrawArrays(GL_LINES, 0, 2 * v_n_edges());
    glPopAttrib();
}

void PetMesh::computeScene()
{
    PetMesh::Point total(0, 0, 0);
    PetMesh::VertexIter v_it, v_end = this->vertices_end();
    for (v_it = this->vertices_begin(); v_it != v_end; ++v_it)
    {
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
    SceneRadius /= 0.618;
}



void PetMesh::setVertexSelected(const unsigned int idx)
{
    if (idx >= n_vertices()) return;
    PetMesh::VertexHandle v_hnd = this->vertex_handle(idx);
    this->set_color(v_hnd, SelectVertexColor);
    this->property(selectedVertex, v_hnd) = true;
}

void PetMesh::setVertexSelected(const PetMesh::VertexHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, SelectVertexColor);
    this->property(selectedVertex, hnd) = true;
}

void PetMesh::setVertexUnselected(const unsigned int idx)
{
    if (idx >= n_vertices()) return;
    PetMesh::VertexHandle v_hnd = this->vertex_handle(idx);
    this->set_color(v_hnd, VertexColor);
    this->property(selectedVertex, v_hnd) = false;
}

void PetMesh::setVertexUnselected(const PetMesh::VertexHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, VertexColor);
    this->property(selectedVertex, hnd) = false;
}

void PetMesh::setVerticesSelected(const std::vector<unsigned int>& idx)
{
    for (std::vector<unsigned int>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setVertexSelected(*it);
    }
}

void PetMesh::setVerticesSelected(const std::vector<PetMesh::VertexHandle>& hnd)
{
    for (std::vector<PetMesh::VertexHandle>::const_iterator it = hnd.begin(); it != hnd.end(); ++it)
    {
        setVertexSelected(*it);
    }
}

void PetMesh::setVerticesUnelected(const std::vector<unsigned int>& idx)
{
    for (std::vector<unsigned int>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setVertexUnselected(*it);
    }
}

void PetMesh::setVerticesUnelected(const std::vector<PetMesh::VertexHandle>& idx)
{
    for (std::vector<PetMesh::VertexHandle>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setVertexUnselected(*it);
    }
}

void PetMesh::setFaceSelected(const unsigned int idx)
{
    if (idx >= n_faces()) return;
    PetMesh::FaceHandle f_hnd = this->face_handle(idx);
    this->set_color(f_hnd, SelectFaceColor);
    this->property(selectedFace, f_hnd) = true;
}

void PetMesh::setFaceSelected(const PetMesh::FaceHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, SelectFaceColor);
    this->property(selectedFace, hnd) = true;
}

void PetMesh::setFaceUnselected(const unsigned int idx)
{
    if (idx >= n_faces()) return;
    PetMesh::FaceHandle f_hnd = this->face_handle(idx);
    this->set_color(f_hnd, FaceColor);
    this->property(selectedFace, f_hnd) = false;
}

void PetMesh::setFaceUnselected(const PetMesh::FaceHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, FaceColor);
    this->property(selectedFace, hnd) = false;
}

void PetMesh::setFacesSelected(const std::vector<unsigned int>& idx)
{
    for (std::vector<unsigned int>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setFaceSelected(*it);
    }
}

void PetMesh::setFacesSelected(const std::vector<PetMesh::FaceHandle>& idx)
{
    for (std::vector<PetMesh::FaceHandle>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setFaceSelected(*it);
    }
}

void PetMesh::setFacesUnselected(const std::vector<unsigned int>& idx)
{
    for (std::vector<unsigned int>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setFaceUnselected(*it);
    }
}

void PetMesh::setFacesUnselected(const std::vector<PetMesh::FaceHandle>& idx)
{
    for (std::vector<PetMesh::FaceHandle>::const_iterator it = idx.begin(); it != idx.end(); ++it)
    {
        setFaceUnselected(*it);
    }
}

void PetMesh::setEdgeSelected(const unsigned int idx)
{
    PetMesh::EdgeHandle e_hnd = this->edge_handle(idx);
    if (!e_hnd.is_valid()) return;
    this->set_color(e_hnd, SelectEdgeColor);
    this->property(selectedEdge, e_hnd) = true;
}

void PetMesh::setEdgeSelected(const PetMesh::EdgeHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, SelectEdgeColor);
    this->property(selectedEdge, hnd) = true;
}

void PetMesh::setEdgesSelected(const std::vector<unsigned int> & idx)
{
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setEdgeSelected(*it);
    }
}

void PetMesh::setEdgesSelected(const std::vector<PetMesh::EdgeHandle> & idx)
{
    std::vector<PetMesh::EdgeHandle>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setEdgeSelected(*it);
    }
}

void PetMesh::setEdgeUnselected(const unsigned int idx)
{
    PetMesh::EdgeHandle e_hnd = this->edge_handle(idx);
    this->set_color(e_hnd, EdgeColor);
    this->property(selectedEdge, e_hnd) = false;
}

void PetMesh::setEdgeUnselected(const PetMesh::EdgeHandle hnd)
{
    if (!hnd.is_valid()) return;
    this->set_color(hnd, EdgeColor);
    this->property(selectedEdge, hnd) = false;
}

void PetMesh::setEdgesUnselected(const std::vector<unsigned int>& idx)
{
    std::vector<unsigned int>::const_iterator it, it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setEdgeUnselected(*it);
    }
}

void PetMesh::setEdgesUnselected(const std::vector<PetMesh::EdgeHandle>& idx)
{
    std::vector<PetMesh::EdgeHandle>::const_iterator it, it_end = idx.end();
    for (; it != it_end; ++it)
    {
        setEdgeUnselected(*it);
    }
}

void PetMesh::getSelectedVertices(std::vector<unsigned int>& idx)
{
    idx.clear();
    PetMesh::VertexIter v_it = vertices_begin(), v_end = vertices_end();
    for(; v_it != v_end; ++v_it)
    {
        if (this->property(selectedVertex, v_it))
            idx.push_back((*v_it).idx());
    }
}

void PetMesh::getSelectedVertices(std::vector<PetMesh::VertexHandle>& hnd)
{
    hnd.clear();
    PetMesh::VertexIter v_it = vertices_begin(), v_end = vertices_end();
    for(; v_it != v_end; ++v_it)
    {
        if (this->property(selectedVertex, v_it))
            hnd.push_back((v_it.handle()));
    }
}

void PetMesh::getSelectedFaces(std::vector<unsigned int>& idx)
{
    idx.clear();
    PetMesh::FaceIter f_it = faces_begin(), f_end = faces_end();
    for(; f_it != f_end; ++f_it)
    {
        if (this->property(selectedFace, f_it))
            idx.push_back((*f_it).idx());
    }
}

void PetMesh::getSelectedFaces(std::vector<PetMesh::FaceHandle>& idx)
{
    idx.clear();
    PetMesh::FaceIter f_it = faces_begin(), f_end = faces_end();
    for(; f_it != f_end; ++f_it)
    {
        if (this->property(selectedFace, f_it))
            idx.push_back(*f_it);
    }
}

void PetMesh::getSelectedEdges(std::vector<unsigned int> & idx)
{
    idx.clear();
    PetMesh::EdgeIter e_it = edges_begin(), e_end = edges_end();
    for (; e_it != e_end; ++e_it)
    {
        if (this->property(selectedEdge, e_it))
            idx.push_back((*e_it).idx());
    }
}

void PetMesh::getSelectedEdges(std::vector<PetMesh::EdgeHandle> & idx)
{
    idx.clear();
    PetMesh::EdgeIter e_it = edges_begin(), e_end = edges_end();
    for (; e_it != e_end; ++e_it)
    {
        if (this->property(selectedEdge, e_it))
            idx.push_back(*e_it);
    }
}

void PetMesh::setVerticesSelectedByFace(const PetMesh::FaceHandle& hnd)
{
    for(PetMesh::FaceVertexIter it = PetMesh::fv_iter(hnd); it; ++it)
    {
        setVertexSelected(it.handle());
    }
}

void PetMesh::setVerticesSelectedByFaces(const std::vector<PetMesh::FaceHandle>& hnd)
{
    std::vector<PetMesh::FaceHandle>::const_iterator it = hnd.begin(), it_end = hnd.end();
    for (; it != it_end; ++it)
    {
        setVerticesSelectedByFace(*it);
    }
}

void PetMesh::clearSelectedEdges()
{
    PetMesh::EdgeIter e_it = edges_begin(), e_end = edges_end();
    for (; e_it != e_end; ++e_it)
    {
        this->property(selectedEdge, e_it) = false;
        this->set_color(e_it, EdgeColor);
    }
}

void PetMesh::clearSelectedVertices()
{
    PetMesh::VertexIter v_it = vertices_begin(), v_end = vertices_end();
    for (; v_it != v_end; ++v_it)
    {
        this->property(selectedVertex, v_it) = false;
        this->set_color(v_it, VertexColor);
    }
}

void PetMesh::clearSelectedFaces()
{
    PetMesh::FaceIter f_it = faces_begin(), f_end = faces_end();
    for (; f_it != f_end; ++f_it)
    {
        this->property(selectedFace, f_it) = false;
        this->set_color(f_it, FaceColor);
    }
}
