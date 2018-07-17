#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QObject>

#include "ProcessHandler.h"

struct Waypoint {
    double lat;
    double lon;
};

class MainWindow : public QObject
{
    Q_OBJECT
public:
    explicit MainWindow(QObject *parent = nullptr);

private:
    ProcessHandler* myHandler;

    QList<int>  vehicles;
    QList<QGeoCoordinate> waypoints;
    QGeoCoordinate depot;
};

#endif // MAINWINDOW_H
