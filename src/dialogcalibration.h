#ifndef DIALOGCALIBRATION_H
#define DIALOGCALIBRATION_H

#include <QDialog>

#include "ckinectcalibration.h"

namespace Ui {
class DialogCalibration;
}

class DialogCalibration : public QDialog
{
    Q_OBJECT
    
public:
    explicit DialogCalibration(QWidget *p = 0, QString image = "/camera/rgb/image_mono");
    ~DialogCalibration();

private:

    QImage IplImage2QImage(const IplImage *iplImage);
    void update_tf();
    void readParams();

public:
    void initGui();
    void terminateAuto();

private slots:
    void updateIR(IplImage* _irImage);
    void updateTF(CvMat* rotMat, CvMat* transMat);

    void on_horizontalSlider_camera_x_valueChanged(int value);
    void on_horizontalSlider_camera_y_valueChanged(int value);
    void on_horizontalSlider_camera_z_valueChanged(int value);
    void on_horizontalSlider_camera_yaw_valueChanged(int value);
    void on_horizontalSlider_camera_pitch_valueChanged(int value);
    void on_horizontalSlider_camera_roll_valueChanged(int value);


    void on_pushButton_start_clicked();

private:
    Ui::DialogCalibration *ui;
    CKinectCalibration *calibration;
    IplImage *irImage;

public:
    double camera_x;
    double camera_y;
    double camera_z;
    double camera_roll;
    double camera_pitch;
    double camera_yaw;
    int checkboard_nW;
    int checkboard_nH;
    double checkboard_size;

private:
    double camera_qx;
    double camera_qy;
    double camera_qz;
    double camera_qw;


signals:
    void emitTF(vector<double> camera_param);
};

#endif // DIALOGCALIBRATION_H
