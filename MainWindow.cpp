#include "MainWindow.h"
#include "QFile"
#include "QCoreApplication"

MainWindow::MainWindow(QObject *parent) : QObject(parent)
{
    vehicles = {1, 2, 3, 4};

    /*
    QFile file(QCoreApplication::applicationDirPath()+"/coords.txt");

    if(!file.open(QIODevice::ReadOnly))
    {
        return;
    }

    QTextStream in(&file);
    while(!in.atEnd())
    {
        QString line = in.readLine();
        line.chop(1);
        QStringList fields = line.split(",");

        double lat = fields[0].toDouble();
        double lon = fields[1].toDouble();

        QGeoCoordinate coord(lat,lon);
        waypoints.append(coord);
    }
    */

    float lat = 33.850847;
    float lon = -118.29500;

    for(int i = 0; i < 20; i++)
    {
        QGeoCoordinate coord(lat, lon);
        waypoints.append(coord);

        if(i % 2 == 0)
            lat += 0.000300;
        else
            lon += 0.000300;

    }


    depot = waypoints[0];
    depot.setLatitude(depot.latitude() - 0.0003);

    myHandler = new ProcessHandler(vehicles, waypoints, depot);
}

