#include "Viewer.h"
using namespace std;

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
    vector<PetMesh*> PetMeshLists = parent->PetMeshLists;
    if (PetMeshLists.empty()) return;
    vector<PetMesh*>::iterator iter;
    for (iter = PetMeshLists.begin(); iter != PetMeshLists.end(); ++iter)
    {
        (*iter)->render();
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
    glEnable(GL_NORMALIZE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    cout << "OpenGL version: " << glGetString(GL_VERSION) <<endl;
    glPrimitiveRestartIndex(UINT_MAX);
    glEnable(GL_PRIMITIVE_RESTART);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
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
