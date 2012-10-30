#include <fstream>

#include <QFileInfo>
#include <QFile>
#include <QString>
#include <QDir>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "PetCurve.h"

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
    this->init(true);

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
        fin >> begin_vertex >> end_vertex;
    }//end while
    fin.close();
    return true;
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
