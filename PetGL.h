#ifndef PETGL_H
#define PETGL_H

#include <vector>

#include <QMainWindow>
#include <QSignalMapper>

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
    int AddPetMesh(PetMesh *petMesh);
    int DeletePetMesh(int num);
    std::vector<PetMesh*> PetMeshLists;

    
private slots:
    void on_actionLoad_mesh_triggered();
    void toggleDrawProperties(QWidget*);

private:
    QSignalMapper *signalMapper;
    QStreamRedirect* qout;
    Ui::MainWindow *ui;
};

#endif // PETGL_H
