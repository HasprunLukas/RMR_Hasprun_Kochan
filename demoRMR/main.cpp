#include "mainwindow.h"
#include <QApplication>
#ifdef _win32
#include "mmsystem.h"
#endif

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
