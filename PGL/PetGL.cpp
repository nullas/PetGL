#include <fstream>
#include <vector>

#include <QString>
#include <QList>
#include <QFileDialog>
#include <QFileInfo>
#include <QListWidgetItem>
#include <QVariant>
#include <QCheckBox>
#include <QSignalMapper>
#include <QApplication>
#include <QPluginLoader>
#include <QTabWidget>
 #include <QTreeWidgetItemIterator>

#include "PetGL.h"
#include "PetMesh.h"
#include "ui_PetGL.h"
#include "PetCurve.h"


using namespace std;


PetGL::PetGL(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->MeshLists->setColumnCount(4);
    QStringList QTreeHeaderNames;
    QTreeHeaderNames << "Mesh" << "V" << "F" << "E" << "P" << "S";
    ui->MeshLists->setHeaderLabels(QTreeHeaderNames);
    for (int i = 0; i < ui->MeshLists->columnCount(); ++i)
        ui->MeshLists->resizeColumnToContents(i);

    signalMapper = new QSignalMapper(this);

    ui->MeshLists->setContextMenuPolicy(Qt::ActionsContextMenu);
    QAction *focusAct = new QAction("focus", ui->MeshLists);
    QAction *saveAct = new QAction("save", ui->MeshLists);
    QAction *deleteAct = new QAction("delete", ui->MeshLists);
    connect(focusAct, SIGNAL(triggered(bool)), this, SLOT(focusPet()));
    connect(saveAct, SIGNAL(triggered(bool)), this, SLOT(savePet()));
    connect(deleteAct, SIGNAL(triggered(bool)), this, SLOT(deletePet()));
    ui->MeshLists->addAction(focusAct);
    ui->MeshLists->addAction(saveAct);
    ui->MeshLists->addAction(deleteAct);

    qout = new QStreamRedirect(std::cout, ui->logWindow);

    PetMeshLists.clear();

    loadPlugins();
}

PetGL::~PetGL()
{
    delete qout;
    delete ui;
    for (vector<PetMesh*>::iterator iter = PetMeshLists.begin(); iter != PetMeshLists.end(); ++ iter)
    {
        delete *iter;
    }
    PetMeshLists.clear();
}

void PetGL::loadPlugins()
{
    QDir pluginsDir(qApp->applicationDirPath());
    foreach (QString fileName, pluginsDir.entryList(QDir::Files))
    {
     QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
     QObject* plugin = loader.instance();
     PetPluginInterface* pplugin;
     if (plugin)
     {
         std::cout << fileName.toStdString() << " loaded" << std::endl;
         pplugin = qobject_cast<PetPluginInterface *>(plugin);
         pplugin->initial(this);
         PluginLists.push_back(pplugin);
     }
    }
}

QTabWidget* PetGL::getPluginTab()
{
    return ui->tabPlugin;
}



int PetGL::AddPetMesh(PetMesh *petMesh)
{
    PetMeshLists.push_back(petMesh);
    cout << "Add mesh: " << petMesh->name.toStdString() << endl;
    cout << "Vertices: " << petMesh->n_vertices() << endl;
    cout << "Faces: " << petMesh->n_faces() << endl;
    cout << "Edges: " << petMesh->n_edges() << endl;

    QTreeWidgetItem *item = new QTreeWidgetItem(ui->MeshLists, QStringList(petMesh->name));
    QVariant V;
    V.setValue((void *)petMesh);
    item->setData(0, Qt::UserRole, V);

    if (ui->MeshLists->topLevelItemCount() == 1)
        ui->MeshLists->setCurrentItem(item);

    QCheckBox *chkBox;

    for (int i = 1; i < ui->MeshLists->columnCount(); ++i)
    {
        chkBox = new QCheckBox();
        ui->MeshLists->setItemWidget(item, i, chkBox);
        if ((i == 2 || i == 5) && petMesh->isCurve)
            chkBox->setCheckable(false);
        else
            chkBox->setCheckable(true);

        if (i < 6 && *petMesh->drawProperties[i - 1])
            chkBox->setChecked(true);
        else
            chkBox->setChecked(false);

        connect(chkBox, SIGNAL(toggled(bool)), signalMapper, SLOT(map()));
        signalMapper->setMapping(chkBox, (QWidget*) item);
    }

    connect(signalMapper, SIGNAL(mapped(QWidget*)), this, SLOT(toggleDrawProperties(QWidget*)));

    for (int i = 0; i < ui->MeshLists->columnCount(); ++i)
        ui->MeshLists->resizeColumnToContents(i);

    ui->MainViewer->updateGL();

    return 0;
}

int PetGL::DeletePetMesh(PetMesh* mesh)
{
    vector<PetMesh*>::iterator it;
    for (it = PetMeshLists.begin(); it != PetMeshLists.end(); ++ it)
    {
        if(*it == mesh) break;
    }
    if (it != PetMeshLists.end())
    {
        std::cout << "Delete Mesh:" << (*it)->name.toStdString() << std::endl;
        delete *it;
        this->PetMeshLists.erase(it);
        return 0;
    }
    else
    {
        std::cout << "Mesh Not found."<< std::endl;
        return 1;
    }
}

void PetGL::on_actionLoad_mesh_triggered()
{
    QString fileName = \
            QFileDialog::getOpenFileName(this, \
                                         tr("Load Mesh"), \
                                         "/home/nullas/workspace/PetGL/meshes", \
                                         tr("Mesh (*.ply *.obj)"));
    if (fileName.isEmpty()) return;
    PetMesh *mesh = new PetMesh();
    mesh->read_mesh(fileName);
    AddPetMesh(mesh);
}


void PetGL::on_actionLoad_curve_triggered()
{
    QString filename = \
            QFileDialog::getOpenFileName(this, \
                                         tr("Load curve"), \
                                         "/home/nullas/workspace/PetGL/meshes", \
                                         tr("curve (*.crv)"));
    if (filename.isEmpty()) return;
    PetCurve *mesh = new PetCurve();
    if(!mesh->read_curve(filename)) return;
    AddPetMesh(mesh);
}




void PetGL::toggleDrawProperties(QWidget* item)
{
    QTreeWidgetItem *chkItem = (QTreeWidgetItem *) item;
    QCheckBox *chkBox;
    PetMesh *mesh;
    mesh = (PetMesh *) chkItem->data(0,Qt::UserRole).value<void *>();

    for (int i = 1; i < ui->MeshLists->columnCount(); ++i)
    {
        chkBox = (QCheckBox *)ui->MeshLists->itemWidget(chkItem, i);
        if (chkBox->isChecked())
            *(mesh->drawProperties[i-1]) = true;
        else
            *(mesh->drawProperties[i-1]) = false;
    }
    ui->MainViewer->updateGL();
}

void PetGL::savePet()
{
    QTreeWidgetItem *item = this->ui->MeshLists->currentItem();
    PetMesh *mesh;
    mesh = (PetMesh *) item->data(0,Qt::UserRole).value<void *>();
    QString filename;

    filename = \
            QFileDialog::getSaveFileName(this, \
                                         tr("Save to"), \
                                         "/home/nullas/workspace/PetGL/meshes",
                                         tr("Mesh (*.obj *.ply *.crv)"));
    if (filename.isEmpty()) return;
    mesh->save(filename);
}


void PetGL::deletePet()
{
    QTreeWidgetItem *item = this->ui->MeshLists->currentItem();
    PetMesh *mesh;
    mesh = (PetMesh *) item->data(0,Qt::UserRole).value<void *>();
    int index = this->ui->MeshLists->indexOfTopLevelItem(item);
    this->ui->MeshLists->takeTopLevelItem(index);
    delete item;
    this->DeletePetMesh(mesh);
    this->ui->MainViewer->updateGL();
}

void PetGL::focusPet()
{
    QTreeWidgetItem *item = this->ui->MeshLists->currentItem();
    PetMesh *mesh;
    mesh = static_cast<PetMesh *>(item->data(0,Qt::UserRole).value<void *>());
    mesh->computeScene();
    this->ui->MainViewer->setSceneRadius(mesh->SceneRadius);
    qglviewer::Vec center;
    center.setValue(mesh->SceneCenter[0],mesh->SceneCenter[1],mesh->SceneCenter[2]);
    this->ui->MainViewer->setSceneCenter(center);
    this->ui->MainViewer->showEntireScene();
}

PetMesh* PetGL::getCurrentMesh()
{
    QTreeWidgetItem *item = this->ui->MeshLists->currentItem();
    if (item == NULL) return NULL;
    PetMesh *mesh;
    mesh = static_cast<PetMesh *>(item->data(0,Qt::UserRole).value<void *>());
    return mesh;
}

bool PetGL::setCurrentMesh(PetMesh* mesh)
{
    QTreeWidgetItemIterator it(ui->MeshLists);
    while (*it)
    {
        if (mesh == static_cast<PetMesh *>((*it)->data(0,Qt::UserRole).value<void *>()))
        {
         ui->MeshLists->setCurrentItem(*it);
         return true;
        }
        ++it;
    }
    return false;
}

void PetGL::updateView(int level)
{
    if (level == 0)
    {
        PetMesh* pmesh = getCurrentMesh();
        if (pmesh)
            pmesh->updateVBO();
    }
    else
    {
        for(std::vector<PetMesh*>::const_iterator it= PetMeshLists.begin(); it != PetMeshLists.end(); ++it)
            (*it)->updateVBO();
    }
    ui->MainViewer->updateGL();
}
