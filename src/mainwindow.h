#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <tf/transform_broadcaster.h>
#include "ckinecttfpublisher.h"
#include "dialogcalibration.h"

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void pubTF();
    void pubTFCalib();

private:
    void readParams();
    void initGui();
    void initStereoAuto();
    void terminateStereoAuto();
    void calQuaternion();

private slots:
    void update_tf(vector<double> camera_param);
    void on_actionCalibration_triggered();


    void on_actionSave_triggered();

    void on_actionSaveAs_triggered();

private:

    Ui::MainWindow *ui;

    QTimer *timer_tf;
    CKinectTFPublisher tf;
    DialogCalibration *digCali;
    QString image;

public:
    QString path;
    QString fileName;
    QString filePath;

    double camera_x;
    double camera_y;
    double camera_z;
    double camera_qx;
    double camera_qy;
    double camera_qz;
    double camera_qw;
    double camera_roll;
    double camera_pitch;
    double camera_yaw;


};

#endif // MAINWINDOW_H
