#ifndef CKINECTTFPUBLISHER_H
#define CKINECTTFPUBLISHER_H

#include <tf/transform_broadcaster.h>
#include "ckinectthread.h"
#include <QTimer>

class CKinectTFPublisher : public CKinectThread
{
    Q_OBJECT
public:
    CKinectTFPublisher();
    void update();

private:
    void pubTFCalib();
    void calQuaternion();
    void init();

private slots:

    void pubtf();

private:
    QTimer *timer;

    tf::TransformBroadcaster br;
    std::vector<tf::StampedTransform> transforms;

    tf::Transform tf_camera_ground;
    tf::Transform tf_camera_link;
    tf::Transform tf_camera_depth_frame;
    tf::Transform tf_camera_rgb_frame;

public:
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

#endif // CKINECTTFPUBLISHER_H
