#ifndef PETGL_H
#define PETGL_H

#include <vector>

#include <GL/glew.h>


#include <QMainWindow>
#include <QSignalMapper>
#include <QMenu>


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
    int DeletePetMesh(PetMesh *);
    std::vector<PetMesh*> PetMeshLists;
    PetMesh* getCurrentMesh();

    
private slots:
    void on_actionLoad_mesh_triggered();
    void on_actionLoad_curve_triggered();
    void toggleDrawProperties(QWidget *);
    void savePet();
    void deletePet();
    void focusPet();


private:
    QSignalMapper *signalMapper;
    QStreamRedirect* qout;
    Ui::MainWindow *ui;
};

#endif // PETGL_H
