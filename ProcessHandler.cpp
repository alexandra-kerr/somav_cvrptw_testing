#include "ProcessHandler.h"
#include <QCoreApplication>

ProcessHandler::ProcessHandler(QList<int> vehicles, QList<QGeoCoordinate> waypoints, QGeoCoordinate depot, QObject *parent) : QObject(parent)
{
    myProcess = new QProcess(this);
    QString program = "python";
    QStringList arguments;
    QString appPath = QCoreApplication::applicationDirPath() + "/somav_cvrptw.py";
    arguments << appPath;

    QString vehiclesList = buildVehiclesList(vehicles);
    QString waypointsList = buildWaypointsList(waypoints);
    qDebug() << waypointsList;
    QString depotStr = "Depot:" + QString::number(depot.latitude()) + "+" + QString::number(depot.longitude());

    arguments << vehiclesList << waypointsList << depotStr;

    myProcess->start(program, arguments);
    connect(myProcess, SIGNAL(stateChanged(QProcess::ProcessState)), this, SLOT(printProcessStatus()));
    connect(myProcess, SIGNAL(error(QProcess::ProcessError)), this, SLOT(printProcessError()));
    connect(myProcess, SIGNAL(readyRead()), this, SLOT(printProcessOutput()));
}

void ProcessHandler::printProcessError()
{
    QString error = myProcess->errorString();
    qDebug() << "Error: " << error;
    emit errorReceived(error);
}

void ProcessHandler::printProcessOutput()
{
    QString out = myProcess->readAll();
    qDebug() << "Proc out: " << out;
    emit outputReceived(out);
}

void ProcessHandler::printProcessStatus()
{
    QProcess::ProcessState state = myProcess->state();
    QString status;
    if(state == 1)
        status = "Starting";
    else if(state == 2)
        status = "Running";
    else
        status = "Not Running";

    qDebug() << status;
    emit statusReceived(status);

    if(state == 0)
    {
        myProcess->close();
    }
}

QString ProcessHandler::buildVehiclesList(QList<int> vehicles)
{
    QString result = "Vehicles:";

    for(int i = 0; i < vehicles.count(); i++)
    {
        if(i > 0)
            result.append(",");
        result.append(QString::number(vehicles[i]));
    }

    return result;
}

QString ProcessHandler::buildWaypointsList(QList<QGeoCoordinate> waypoints)
{
    QString result = "Waypoints:";

    for(int i = 0; i < 20; i++)
    {
        if(i > 0)
            result.append(",");
        result.append(QString::number(waypoints[i].latitude()) + "+" + QString::number(waypoints[i].longitude()));
    }

    return result;
}
