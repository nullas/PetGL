#ifndef PETGL_H
#define PETGL_H

#include <list>

#include <QMainWindow>
#include <QGLViewer/qglviewer.h>

#include "PetMesh.h"
#include "QStreamRedirect.h"

namespace Ui {
class MainWindow;
}

class PetGL : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit PetGL(QWidget *parent = 0);
    ~PetGL();
    int AddPetMesh(PetMesh petMesh);
    int DeletePetMesh(int num);
    std::list<PetMesh*> PetMeshLists;

    
private slots:
    void on_actionLoad_mesh_triggered();

private:
    QStreamRedirect* qout;
    Ui::MainWindow *ui;
};

#endif // PETGL_H
