#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QFileDialog>

#define pi 3.141592

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    digCali = 0;
    image.append("/camera/rgb/image_raw");
    ros::start();
    ros::NodeHandle n;

    ui->setupUi(this);
    readParams();
    initGui();
    pubTFCalib();

}

MainWindow::~MainWindow()
{
    if(digCali)
        digCali->terminateAuto();

    delete ui;
}

void MainWindow::readParams()
{
    bool isOK = 1;
    if(ros::param::has("/dhri/calibrationOneCam/camera/x"))
        ros::param::get("/dhri/calibrationOneCam/camera/x", camera_x);
    else isOK = 0;
    if(ros::param::has("/dhri/calibrationOneCam/camera/y"))
        ros::param::get("/dhri/calibrationOneCam/camera/y", camera_y);
    else isOK = 0;
    if(ros::param::has("/dhri/calibrationOneCam/camera/z"))
        ros::param::get("/dhri/calibrationOneCam/camera/z", camera_z);
    else isOK = 0;
    if(ros::param::has("/dhri/calibrationOneCam/camera/yaw"))
        ros::param::get("/dhri/calibrationOneCam/camera/yaw", camera_yaw);
    else isOK = 0;
    if(ros::param::has("/dhri/calibrationOneCam/camera/pitch"))
        ros::param::get("/dhri/calibrationOneCam/camera/pitch", camera_pitch);
    else isOK = 0;
    if(ros::param::has("/dhri/calibrationOneCam/camera/roll"))
        ros::param::get("/dhri/calibrationOneCam/camera/roll", camera_roll);
    else isOK = 0;


    if(isOK){
    }
    else{
        qDebug("Run 'rosparam load param_camera.yaml' first");
        return;
    }
}

void MainWindow::initGui()
{
    ui->lineEdit_x->setText(QString::number(camera_x));
    ui->lineEdit_y->setText(QString::number(camera_y));
    ui->lineEdit_z->setText(QString::number(camera_z));
    ui->lineEdit_roll->setText(QString::number(camera_roll));
    ui->lineEdit_pitch->setText(QString::number(camera_pitch));
    ui->lineEdit_yaw->setText(QString::number(camera_yaw));
}

void MainWindow::pubTFCalib()
{
    tf.camera_x = camera_x;
    tf.camera_y = camera_y;
    tf.camera_z = camera_z;
    tf.camera_roll = camera_roll;
    tf.camera_pitch = camera_pitch;
    tf.camera_yaw = camera_yaw;

    tf.update();
}

void MainWindow::update_tf(vector<double> camera_param)
{
    camera_x = camera_param.at(0);
    camera_y = camera_param.at(1);
    camera_z = camera_param.at(2);
    camera_roll = camera_param.at(3);
    camera_pitch = camera_param.at(4);
    camera_yaw = camera_param.at(5);
    initGui();
    pubTFCalib();
}

void MainWindow::on_actionCalibration_triggered()
{
    if(ui->actionCalibration->isChecked()){
        digCali = new DialogCalibration(this, image);
        connect(digCali, SIGNAL(emitTF(vector<double>)), this, SLOT(update_tf(vector<double>)));
        digCali->camera_x = camera_x;
        digCali->camera_y = camera_y;
        digCali->camera_z = camera_z;
        digCali->camera_roll = camera_roll;
        digCali->camera_pitch = camera_pitch;
        digCali->camera_yaw = camera_yaw;
        digCali->initGui();
        digCali->show();
    }
    else{
        digCali->terminateAuto();
        digCali->close();
    }
}
#include <QDebug>

void MainWindow::on_actionSave_triggered()
{
    QFile file(path + "/" + fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << "/dhri/calibrationOneCam/camera/x: " << camera_x << "\n";
    out << "/dhri/calibrationOneCam/camera/y: " << camera_y << "\n";
    out << "/dhri/calibrationOneCam/camera/z: " << camera_z << "\n";
    out << "/dhri/calibrationOneCam/camera/yaw: " << camera_yaw << "\n";
    out << "/dhri/calibrationOneCam/camera/pitch: " << camera_pitch << "\n";
    out << "/dhri/calibrationOneCam/camera/roll: " << camera_roll ;
    file.close();

    qDebug() << "The current parameters are saved to the default file : "<< path + "/" + fileName;

}

void MainWindow::on_actionSaveAs_triggered()
{
    QString filePath = QFileDialog::getSaveFileName(this,
                                                    tr("Save camera parameters as"), path, tr("yaml Files (*.yaml)"));

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << "/dhri/calibrationOneCam/x: " << camera_x << "\n";
    out << "/dhri/calibrationOneCam/y: " << camera_y << "\n";
    out << "/dhri/calibrationOneCam/z: " << camera_z << "\n";
    out << "/dhri/calibrationOneCam/yaw: " << camera_yaw << "\n";
    out << "/dhri/calibrationOneCam/pitch: " << camera_pitch << "\n";
    out << "/dhri/calibrationOneCam/roll: " << camera_roll ;
    file.close();

    qDebug() << "The current parameters are saved to the default file : "<< filePath;
}
