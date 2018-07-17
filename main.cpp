#include <QCoreApplication>

#include "MainWindow.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    MainWindow *app = new MainWindow();

    return a.exec();
}
