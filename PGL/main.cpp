#include <QApplication>

#include "PetGL.h"


int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    PetGL petGL;
    petGL.show();
    
    return application.exec();
}
