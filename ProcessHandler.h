#ifndef PROCESSHANDLER_H
#define PROCESSHANDLER_H

#include <QObject>
#include <QProcess>
#include <QDebug>
#include <QGeoCoordinate>

class ProcessHandler : public QObject
{
    Q_OBJECT
public:
    ProcessHandler(QList<int> vehicles, QList<QGeoCoordinate> waypoints, QGeoCoordinate depot, QObject *parent = nullptr);

Q_SIGNALS:
    void outputReceived(QString output);
    void statusReceived(QString status);
    void errorReceived(QString error);

public Q_SLOTS:
    void printProcessStatus();
    void printProcessError();
    void printProcessOutput();

private:
    QString buildVehiclesList(QList<int> vehicles);
    QString buildWaypointsList(QList<QGeoCoordinate> waypoints);

    QProcess* myProcess;

};

#endif // MAINWINDOW_H
