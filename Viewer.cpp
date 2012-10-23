#include <vector>

#include "PetGL.h"
#include "Viewer.h"
#include "PetMesh.h"

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
        drawMesh(**iter);
    }
}

void Viewer::init()
{
    setBackgroundColor(Qt::white);
    glEnable(GL_NORMALIZE);
    glEnable( GL_LINE_SMOOTH );
    glEnable( GL_POLYGON_SMOOTH );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
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
                    glColor3dv(petMesh.color(f_it).data());
                    for (fv_it = petMesh.fv_iter(*f_it); fv_it; ++fv_it)
                    {
                        glNormal3dv(petMesh.normal(fv_it).data());
                        glVertex3dv(petMesh.point(fv_it).data());
                    }
                    glEnd();
                }// end for
            else
                for (f_it = petMesh.faces_begin(); f_it != petMesh.faces_end(); ++f_it)
                {
                    glBegin(GL_POLYGON);
                    glNormal3dv(petMesh.normal(f_it).data());
                    glColor4dv(petMesh.color(f_it).data());
                    for (fv_it = petMesh.fv_iter(*f_it); fv_it; ++fv_it)
                    {
                        glVertex3dv(petMesh.point(fv_it).data());
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
                he_hnd = petMesh.halfedge_handle(e_it.handle(),0);
                glColor3dv(petMesh.color(e_it).data());
                glVertex3dv(petMesh.point(petMesh.from_vertex_handle(he_hnd)).data());
                glVertex3dv(petMesh.point(petMesh.to_vertex_handle(he_hnd)).data());
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
                glColor3dv(petMesh.color(v_it).data());
                glVertex3dv(petMesh.point(v_it).data());
            }//end for
            glEnd();
            glEnable(GL_LIGHTING);
        }

    }// endif (petMesh.visible)
}
