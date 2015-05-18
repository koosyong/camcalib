#include "ckinecttfpublisher.h"
#include <math.h>
#define pi 3.141592

CKinectTFPublisher::CKinectTFPublisher()
{
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), SLOT(pubtf()));
    timer->start(1);
    start();
}

void CKinectTFPublisher::init()
{
    ros::start();
    ros::NodeHandle n;
    exec();
}

void CKinectTFPublisher::update()
{
    pubTFCalib();    
}

void CKinectTFPublisher::pubtf()
{
    ros::Time now = ros::Time::now();
    br.sendTransform(tf::StampedTransform (tf_camera_link, now, "/origin", "/camera_link"));
    br.sendTransform(tf::StampedTransform (tf_camera_depth_frame, now, "/camera_link", "camera_depth_frame"));
    br.sendTransform(tf::StampedTransform (tf_camera_rgb_frame, now, "/camera_link", "/camera_rgb_frame"));

//    qDebug("%d, tf broadcasting", now.toNSec());

}

void CKinectTFPublisher::pubTFCalib()
{
    calQuaternion();
    tf_camera_link.setOrigin( tf::Vector3(camera_x, camera_y, camera_z) );
    tf_camera_link.setRotation( tf::Quaternion(camera_qx, camera_qy, camera_qz, camera_qw ));
    tf_camera_depth_frame.setOrigin( tf::Vector3(0.0, -0.02, 0.0) );
    tf_camera_depth_frame.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ));
    //    tf_camera1_depth_frame.setRotation( tf::Quaternion( pi/2, 0.0, -pi/2 ));
    tf_camera_rgb_frame.setOrigin( tf::Vector3(0.0, -0.045, 0.0) );
    tf_camera_rgb_frame.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ));
    //    tf_camera1_rgb_frame.setRotation( tf::Quaternion( pi/2, 0.0, -pi/2 ));

}


void CKinectTFPublisher::calQuaternion()
{
    double roll, pitch, yaw;
    double q0,q1,q2,q3;

    roll = camera_roll/180*pi;
    pitch = camera_pitch/180*pi;
    yaw = camera_yaw/180*pi;
    q0 = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    q1 = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    q2 = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    q3 = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);

    camera_qw = q0;
    camera_qx = q1;
    camera_qy = q2;
    camera_qz = q3;


}
