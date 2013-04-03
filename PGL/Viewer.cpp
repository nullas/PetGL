#include "Viewer.h"

const int Viewer::PickRadius = 10;


Viewer::Viewer(QWidget* parent)
    : QGLViewer(parent)
{
    selecttype = 0;
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
    std::vector<PetPluginInterface *>::const_iterator it = parent->PluginLists.begin(),
            it_end = parent->PluginLists.end();
    for (; it != it_end; ++it)
    {
        (*it)->drawExtra();
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

void Viewer::keyPressEvent(QKeyEvent *e)
{
    // Defines the Alt+R shortcut.
    if (e->key() == Qt::Key_Space)
    {
        selecttype = (selecttype + 1) % 2;
        std::cout << (selecttype == 0 ? "vertex" : "edge") << std::endl;
    }
    else
    QGLViewer::keyPressEvent(e);
}

void Viewer::Select(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    makeCurrent();
    this->camera()->loadProjectionMatrix();
    this->camera()->loadModelViewMatrix();
    switch (selecttype)
    {
    case 0:
        drawPickVertices();
        pickVertices(e);
        break;
    case 1:
        drawPickEdges();
        pickEdges(e);
        break;
    }
    return;
}
void Viewer::drawPickVertices()
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    QColor qcolor = backgroundColor();
    setBackgroundColor(Qt::white);
    mesh->drawPickVertices();
    setBackgroundColor(qcolor);

}

void Viewer::drawPickEdges()
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    QColor qcolor = backgroundColor();
    setBackgroundColor(Qt::white);
    if (mesh == NULL) return;
    mesh->drawPickEdges();
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
    std::cout << idx << std::endl;
    mesh->set_color(mesh->vertex_handle(idx),PetMesh::SelectVertexColor);
    mesh->property(mesh->selectedVertex, mesh->vertex_handle(idx)) = true;
    mesh->updateVBO();
    updateGL();
}


void Viewer::pickEdges(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    int screenheight = this->camera()->screenHeight();
    int x = e->x(), y = screenheight - e->y();
    if (x < PickRadius || y < PickRadius || x > screenheight - PickRadius || y > screenheight - PickRadius)
        return;
    unsigned int n_edges = mesh->n_edges();
    unsigned char* m_Pick = new unsigned char[12 * PickRadius * PickRadius];
    glReadPixels(x - PickRadius, y - PickRadius, 2 * PickRadius, 2 * PickRadius, GL_RGB,GL_UNSIGNED_BYTE, m_Pick);
    long int idx = -1,tmpidx;
    int base = 0, jbase = 0, radius = INT_MAX;
    for (int i = -PickRadius; i < PickRadius; ++i)
    {

        for (int j = -PickRadius; j < PickRadius; ++j)
            {
            tmpidx = m_Pick[base + jbase] + m_Pick[base + jbase + 1] * 256 + m_Pick[base + jbase + 2] * 65536;
            if(tmpidx < n_edges && i * i + j * j < radius)
            {
                idx = tmpidx;
                radius = i * i + j * j;
            }
            jbase += 3;
        }
        base += 6 * PickRadius;
        jbase = 0;
    }
    if (idx >= n_edges || idx == -1) return;
    std::cout << idx << std::endl;
    mesh->setEdgeSelected(idx);
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
    switch (selecttype)
    {
    case 0:
        drawPickVertices();
        unpickVertices(e);
        break;
    case 1:
        drawPickEdges();
        unpickEdges(e);
        break;

    }
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
    mesh->updateVBO();
    updateGL();
}

void Viewer::unpickEdges(QMouseEvent* e)
{
    PetMesh* mesh = qobject_cast<PetGL*>(this->parent())->getCurrentMesh();
    if (mesh == NULL) return;
    int screenheight = this->camera()->screenHeight();
    int x = e->x(), y = screenheight - e->y();
    if (x < PickRadius || y < PickRadius || x > screenheight - PickRadius || y > screenheight - PickRadius)
        return;
    unsigned int n_edges = mesh->n_edges();
    unsigned char* m_Pick = new unsigned char[12 * PickRadius * PickRadius];
    glReadPixels(x - PickRadius, y - PickRadius, 2 * PickRadius, 2 * PickRadius, GL_RGB,GL_UNSIGNED_BYTE, m_Pick);
    long int idx = -1,tmpidx;
    int base = 0, jbase = 0, radius = INT_MAX;
    for (int i = -PickRadius; i < PickRadius; ++i)
    {

        for (int j = -PickRadius; j < PickRadius; ++j)
            {
            tmpidx = m_Pick[base + jbase] + m_Pick[base + jbase + 1] * 256 + m_Pick[base + jbase + 2] * 65536;
            if(tmpidx < n_edges && i * i + j * j < radius)
            {
                idx = tmpidx;
                radius = i * i + j * j;
            }
            jbase += 3;
        }
        base += 6 * PickRadius;
        jbase = 0;
    }
    if (idx >= n_edges || idx == -1) return;
    mesh->setEdgeUnselected(idx);
    mesh->updateVBO();
    updateGL();
}


