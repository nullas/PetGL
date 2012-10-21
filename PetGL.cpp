#include "PetGL.h"
#include "ui_PetGL.h"

#include <list>

#include <QFileDialog>
#include <QFileInfo>

#include <QGLViewer/qglviewer.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include "PetMesh.h"




using namespace std;


PetGL::PetGL(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qout = new QStreamRedirect(std::cout, ui->logWindow);

    PetMeshLists.clear();
}

PetGL::~PetGL()
{
    delete qout;
    delete ui;
}

int PetGL::AddPetMesh(PetMesh petMesh)
{
    PetMeshLists.push_back(&petMesh);
    std::cout << "Add mesh:" << petMesh.name.toStdString() << std::endl;
    return 0;
}

int PetGL::DeletePetMesh(int num)
{
    list<PetMesh*>::iterator it;
    int found = 0;
    PetMesh* temppointer;
    for (it = PetMeshLists.begin(); it != PetMeshLists.end(); ++ it)
    {
        if((*it)->Identity == num)
        {
            found = 1;
            temppointer = *it;
            PetMeshLists.erase(it);
            break;
        }
    }
    if (found)
    {
        std::cout << "Delete Mesh:" << temppointer->name.toStdString() << std::endl;
        delete temppointer;
    }
    else
        std::cout << "Not found by Identity:" << num << std::endl;

    return found;

}

void PetGL::on_actionLoad_mesh_triggered()
{
    QString fileName = \
            QFileDialog::getOpenFileName(this, \
                                         tr("Load Mesh"), \
                                         tr("Mesh (*.ply *.obj)"));
    if (fileName.isEmpty()) return;
    PetMesh mesh;
    if (!OpenMesh::IO::read_mesh(mesh, fileName.toStdString())) return;
    QFileInfo fi(fileName);
    mesh.SetName(fi.fileName());
    AddPetMesh(mesh);
}
