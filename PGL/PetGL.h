#ifndef PETGL_H
#define PETGL_H

#include <vector>

#include <GL/glew.h>


#include <QMainWindow>
#include <QSignalMapper>
#include <QMenu>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


#include "PetMesh.h"
#include "PetCurve.h"
#include "QStreamRedirect.h"
#include "PluginInterface.h"

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
    bool setCurrentMesh(PetMesh* mesh);
    QTabWidget* getPluginTab();
    std::vector<PetPluginInterface *> PluginLists;


public slots:
    void updateView(int level=0);
    
private slots:
    void on_actionLoad_mesh_triggered();
    void on_actionLoad_curve_triggered();
    void toggleDrawProperties(QWidget *);
    void loadPlugins();
    void savePet();
    void deletePet();
    void focusPet();
    void dumpToCSVFile();


private:
    QSignalMapper *signalMapper;
    QStreamRedirect* qout;
    Ui::MainWindow *ui;

};

#endif // PETGL_H
