
#include <ros/ros.h>

#include "mainwindow.h"
#include <QtGui>

int main (int argc, char** argv)
{
    ros::init (argc, argv, "dhri_calibrationOneCam");

    QApplication a(argc, argv);
    MainWindow view;

    view.path = QString(argv[1]);
    view.fileName = QString(argv[2]);
//    if(QString(argv[3]).compare("on")==0)

        view.show();

    return a.exec();
}
