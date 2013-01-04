#include "Viewer.h"

const int Viewer::PickRadius = 10;


Viewer::Viewer(QWidget* parent)
    : QGLViewer(parent)
{
}


void Viewer::draw()
{

    PetGL* parent = qobject_cast<PetGL*>(this->parent());
    if (parent == NULL)
    {
        cout<<"qobject_cast got wrong"<<endl;
        return;
    }
    vector<PetMesh*> &PetMeshLists = parent->PetMeshLists;
    if (PetMeshLists.empty()) return;
    vector<PetMesh*>::iterator iter;
    for (iter = PetMeshLists.begin(); iter != PetMeshLists.end(); ++iter)
    {
        if ((*iter)->visible) (*iter)->render();
//        drawMesh(**iter);
    }
}

void Viewer::init()
{
    GLenum err = glewInit();
    if (err != GLEW_OK)
        cout << "GLEW got wrong!" << endl;
    else
        cout << "GLEW initialed" << endl;
    setBackgroundColor(Qt::white);
    setForegroundColor(Qt::black);
    glEnable(GL_NORMALIZE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    cout << "OpenGL version: " << glGetString(GL_VERSION) <<endl;
    glPrimitiveRestartIndex(UINT_MAX);
    glEnable(GL_PRIMITIVE_RESTART);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

//#if QT_VERSION < 0x040000
//  // Define Control+Shift+Right button as selection shortcut
//  setMouseBinding(Qt::ControlButton | Qt::LeftButton, SELECT);
//#else
//  setMouseBinding(Qt::ControlModifier | Qt::LeftButton, SELECT);
//#endif
}

void Viewer::drawMesh(PetMesh& petMesh)
{
    if (petMesh.visible)
    {
        if (petMesh.showFaces)
        {
            PetMesh::ConstFaceIter f_it;
            PetMesh::ConstFaceVertexIter fv_it;
            if (petMesh.smooth)
                for (f_it = petMesh.faces_begin(); f_it != petMesh.faces_end(); ++f_it)
                {
                    glBegin(GL_POLYGON);
                    glColor3fv(petMesh.color(f_it).data());
                    for (fv_it = petMesh.fv_iter(*f_it); fv_it; ++fv_it)
                    {
                        glNormal3fv(petMesh.normal(fv_it).data());
                        glVertex3fv(petMesh.point(fv_it).data());
                    }
                    glEnd();
                }// end for
            else
                for (f_it = petMesh.faces_begin(); f_it != petMesh.faces_end(); ++f_it)
                {
                    glBegin(GL_POLYGON);
                    glNormal3fv(petMesh.normal(f_it).data());
                    glColor4fv(petMesh.color(f_it).data());
                    for (fv_it = petMesh.fv_iter(*f_it); fv_it; ++fv_it)
                    {
                        glVertex3fv(petMesh.point(fv_it).data());
                    }
                    glEnd();
                }// end for
        }// endif (petMesh.showFaces)
        if (petMesh.showEdges)
        {
                glDisable(GL_LIGHTING);
                PetMesh::EdgeIter e_it;
                PetMesh::HalfedgeHandle he_hnd;
                glLineWidth(2.0);
                glBegin(GL_LINES);
                for (e_it = petMesh.edges_begin(); e_it != petMesh.edges_end(); ++e_it)
                {
                    if (petMesh.property(petMesh.showEdge, *e_it))
                    {
                        he_hnd = petMesh.halfedge_handle(e_it.handle(),0);
                        glColor3fv(petMesh.color(e_it).data());
                        glVertex3fv(petMesh.point(petMesh.from_vertex_handle(he_hnd)).data());
                        glVertex3fv(petMesh.point(petMesh.to_vertex_handle(he_hnd)).data());
                    }
                }//end for
                glEnd();
                glEnable(GL_LIGHTING);
        }//endif (petMesh.showEdges)
        if (petMesh.showVertices)
        {
            glDisable(GL_LIGHTING);
            PetMesh::VertexIter v_it;
            glPointSize(4.0);
            glBegin(GL_POINTS);
            for (v_it = petMesh.vertices_begin(); v_it != petMesh.vertices_end(); ++v_it)
            {
                glColor3fv(petMesh.color(v_it).data());
                glVertex3fv(petMesh.point(v_it).data());
            }//end for
            glEnd();
            glEnable(GL_LIGHTING);
        }

    }// endif (petMesh.visible)
}


void Viewer::mousePressEvent(QMouseEvent* e)
{
    bool actioned = false;
#if QT_VERSION < 0x040000
  if ((e->button() == Qt::LeftButton) && (e->state() == Qt::CTRL))
#else
  if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::CTRL))
#endif
  {
      Select(e);
      actioned = true;
  }
#if QT_VERSION < 0x040000
  if ((e->button() == Qt::LeftButton) && (e->state() == Qt::SHIFT))
#else
  if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::SHIFT))
#endif
  {
      Deselect(e);
      actioned = true;
  }
  if (!actioned) QGLViewer::mousePressEvent(e);
}

void Viewer::Select(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    makeCurrent();
    this->camera()->loadProjectionMatrix();
    this->camera()->loadModelViewMatrix();
    drawPickVertices();
    pickVertices(e);
    return;
}
void Viewer::drawPickVertices()
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    QColor qcolor = backgroundColor();
    setBackgroundColor(Qt::white);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f(1.,1.,1.);
    glBindBuffer(GL_ARRAY_BUFFER, mesh->bufferObjs[0]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->bufferObjs[2]);
    glDrawElements(GL_POLYGON, mesh->iSizeofidxFaces + mesh->n_faces(), GL_UNSIGNED_INT, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    unsigned int n_vertices = mesh->n_vertices();
    unsigned char* pickColor = new unsigned char[4 * n_vertices];
    unsigned char r=0,g=0,b=0;
    long int i = 0, tmpidx;
    PetMesh::VertexIter v_end(mesh->vertices_end());
    for (PetMesh::VertexIter v_it = mesh->vertices_begin(); v_it != v_end; ++v_it)
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
    glBindBuffer(GL_ARRAY_BUFFER, mesh->bufferObjs[8]);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(unsigned char) * n_vertices, pickColor, GL_DYNAMIC_DRAW);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);

    delete [] pickColor;

    glBindBuffer(GL_ARRAY_BUFFER, mesh->bufferObjs[7]);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glPointSize(2.0);
    glDrawArrays(GL_POINTS, 0, n_vertices);
    glPopAttrib();
    setBackgroundColor(qcolor);

}


void Viewer::pickVertices(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    int screenheight = this->camera()->screenHeight();
    int x = e->x(), y = screenheight - e->y();
    if (x < PickRadius || y < PickRadius || x > screenheight - PickRadius || y > screenheight - PickRadius)
        return;
    unsigned int n_vertices = mesh->n_vertices();
    unsigned char* m_Pick = new unsigned char[12 * PickRadius * PickRadius];
    glReadPixels(x - PickRadius, y - PickRadius, 2 * PickRadius, 2 * PickRadius, GL_RGB,GL_UNSIGNED_BYTE, m_Pick);
    long int idx = -1,tmpidx;
    int base = 0, jbase = 0, radius = INT_MAX;
    for (int i = -PickRadius; i < PickRadius; ++i)
    {

        for (int j = -PickRadius; j < PickRadius; ++j)
            {
            tmpidx = m_Pick[base + jbase] + m_Pick[base + jbase + 1] * 256 + m_Pick[base + jbase + 2] * 65536;
            if(tmpidx < n_vertices && i * i + j * j < radius)
            {
                idx = tmpidx;
                radius = i * i + j * j;
            }
            jbase += 3;
        }
        base += 6 * PickRadius;
        jbase = 0;
    }
    if (idx >= n_vertices || idx == -1) return;
    mesh->set_color(mesh->vertex_handle(idx),PetMesh::SelectVertexColor);
    mesh->property(mesh->selectedVertex, mesh->vertex_handle(idx)) = true;
    mesh->showVertices = true;
    mesh->updateVBO();
    updateGL();
}


void Viewer::Deselect(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    makeCurrent();
    this->camera()->loadProjectionMatrix();
    this->camera()->loadModelViewMatrix();
    drawPickVertices();
    unpickVertices(e);
    return;

}

void Viewer::unpickVertices(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    int screenheight = this->camera()->screenHeight();
    int x = e->x(), y = screenheight - e->y();
    if (x < PickRadius || y < PickRadius || x > screenheight - PickRadius || y > screenheight - PickRadius)
        return;
    unsigned int n_vertices = mesh->n_vertices();
    unsigned char* m_Pick = new unsigned char[12 * PickRadius * PickRadius];
    glReadPixels(x - PickRadius, y - PickRadius, 2 * PickRadius, 2 * PickRadius, GL_RGB,GL_UNSIGNED_BYTE, m_Pick);
    long int idx = -1,tmpidx;
    int base = 0, jbase = 0, radius = INT_MAX;
    for (int i = -PickRadius; i < PickRadius; ++i)
    {

        for (int j = -PickRadius; j < PickRadius; ++j)
            {
            tmpidx = m_Pick[base + jbase] + m_Pick[base + jbase + 1] * 256 + m_Pick[base + jbase + 2] * 65536;
            if(tmpidx < n_vertices && i * i + j * j < radius)
            {
                idx = tmpidx;
                radius = i * i + j * j;
            }
            jbase += 3;
        }
        base += 6 * PickRadius;
        jbase = 0;
    }
    if (idx >= n_vertices || idx == -1) return;
    mesh->set_color(mesh->vertex_handle(idx),PetMesh::VertexColor);
    mesh->property(mesh->selectedVertex, mesh->vertex_handle(idx)) = false;
    mesh->showVertices = true;
    mesh->updateVBO();
    updateGL();
}

