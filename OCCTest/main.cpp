#include "test.h"
#include <QtWidgets/QApplication>
#include "MainWindow.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //MainWindow w;
    test w;
    w.show();
     
    return a.exec();

}
