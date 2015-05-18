#include "ckinectcalibration.h"

#include <stdio.h>
#include <stdlib.h>

#include <QtGui>


CKinectCalibration::CKinectCalibration(string _inputTopic, int nW, int nH, double size, int _maxIter)
    : inputTopic(_inputTopic), maxIter(_maxIter)
{
    gray_image = cvCreateImage(cvSize(1280, 960),IPL_DEPTH_8U,1);
    IRImage = cvCreateImage(cvSize(1280,960),IPL_DEPTH_8U,3);
    isStarted = 0;
    cnt = 0;
    successes = 0;

    board_w = nW;
    board_h = nH;
    board_width = size;

    board_n = board_w * board_h;
    image_points = cvCreateMat(board_n*maxIter,2,CV_32FC1);
    object_points = cvCreateMat(board_n*maxIter,3,CV_32FC1);

    intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
    distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
    intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
    distortion_coeffs = cvCreateMat(5,1,CV_32FC1);

    if(inputTopic.compare("/camera/rgb/image_mono") == 0){
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 516.36431807884; //initializing intrinsic parameters
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 2) = 324.748839605627;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 517.226728137528;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 2) = 248.041687767885;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 2) = 1.0;
        CV_MAT_ELEM(*distortion_coeffs, float, 0, 0) = 0.158515970479281;
        CV_MAT_ELEM(*distortion_coeffs, float, 1, 0) = -0.304062583717913;
        CV_MAT_ELEM(*distortion_coeffs, float, 2, 0) = -0.00258286894865001;
        CV_MAT_ELEM(*distortion_coeffs, float, 3, 0) = -0.00140169070429279;
        CV_MAT_ELEM(*distortion_coeffs, float, 4, 0) = 0.000000;
    }
    if(inputTopic.compare("/camera2/rgb/image_mono") == 0){
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 515.667530465788; //initializing intrinsic parameters
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 2) = 327.147280004659;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 517.509031587659;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 2) = 228.241264503092;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 2) = 1.0;
        CV_MAT_ELEM(*distortion_coeffs, float, 0, 0) = 0.164960813521528;
        CV_MAT_ELEM(*distortion_coeffs, float, 1, 0) = -0.333100916269347;
        CV_MAT_ELEM(*distortion_coeffs, float, 2, 0) = -0.00150554365820593;
        CV_MAT_ELEM(*distortion_coeffs, float, 3, 0) = 0.00082190033233914;
        CV_MAT_ELEM(*distortion_coeffs, float, 4, 0) = 0.000000;
    }
    if(inputTopic.compare("/camera/ir/image_raw") == 0){
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 570.562378578486; //initializing intrinsic parameters
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 2) = 321.850984216972;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 572.069009459035;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 2) = 246.913942765339;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 2) = 1.0;
        CV_MAT_ELEM(*distortion_coeffs, float, 0, 0) = -0.0898747344488993;
        CV_MAT_ELEM(*distortion_coeffs, float, 1, 0) = 0.179992608534601;
        CV_MAT_ELEM(*distortion_coeffs, float, 2, 0) = 0.00232481697723621;
        CV_MAT_ELEM(*distortion_coeffs, float, 3, 0) = -0.00125108853174995;
        CV_MAT_ELEM(*distortion_coeffs, float, 4, 0) = 0.000000;
    }
    if(inputTopic.compare("/camera2/ir/image_raw") == 0){
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 583.797755040541; //initializing intrinsic parameters
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 0, 2) = 321.899749338058;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 585.722069332777;
        CV_MAT_ELEM(*intrinsic_matrix, float, 1, 2) = 242.726555130109;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 0) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 1) = 0.0;
        CV_MAT_ELEM(*intrinsic_matrix, float, 2, 2) = 1.0;
        CV_MAT_ELEM(*distortion_coeffs, float, 0, 0) = -0.073644671724626;
        CV_MAT_ELEM(*distortion_coeffs, float, 1, 0) = 0.165546706270743;
        CV_MAT_ELEM(*distortion_coeffs, float, 2, 0) = -0.000708553326560646;
        CV_MAT_ELEM(*distortion_coeffs, float, 3, 0) = 0.000573020597468945;
        CV_MAT_ELEM(*distortion_coeffs, float, 4, 0) = 0.000000;
    }
}

void CKinectCalibration::image_cb (const sensor_msgs::ImageConstPtr& input)
{
    for (int y = 0;y < input->height; y++)
    {
        for(int x = 0;x < input->width; x++)
        {
            // ir
//            char data8 = (input->data[y*input->step+x*2] | (input->data[y*input->step+x*2+1] << 8));
            // rgb
            char data8 = input->data[y*input->step+x];
            IRImage->imageData[(y*input->width+x)*3] = data8;
            IRImage->imageData[(y*input->width+x)*3+1] = data8;
            IRImage->imageData[(y*input->width+x)*3+2] = data8;
        }
    }


    CvSize board_sz = cvSize(board_w, board_h);
    CvPoint2D32f* corners = new CvPoint2D32f[board_n];
    int corner_count;

    int step, frame = 0;

    //    CvMat* point_counts = cvCreateMat(1,1,CV_32SC1);
    CvMat* rvec = cvCreateMat(3,1,CV_32FC1);
    CvMat* tvec = cvCreateMat(3,1,CV_32FC1);

    if(isStarted){
        cnt ++;

        if (successes < maxIter )
        {
            int found  = cvFindChessboardCorners(IRImage, board_sz, corners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

            cvCvtColor(IRImage, gray_image, CV_BGR2GRAY);
            cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11,11), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

            cvDrawChessboardCorners(IRImage, board_sz, corners, corner_count, found);

            if (corner_count == board_n)
            {
                for (int j = 0; j < board_n;++j)
                {
                    CV_MAT_ELEM(*image_points, float, successes*board_n+j, 0) = corners[j].x;
                    CV_MAT_ELEM(*image_points, float, successes*board_n+j, 1) = corners[j].y;
                    CV_MAT_ELEM(*object_points, float, successes*board_n+j, 0) = j/board_w*board_width;
                    CV_MAT_ELEM(*object_points, float, successes*board_n+j, 1) = j%board_w*board_width;
                    CV_MAT_ELEM(*object_points, float, successes*board_n+j, 2) = 0.0f;
                }
                //                CV_MAT_ELEM(*point_counts, int, 0, 0) = board_n;
                successes++;
                qDebug("%s SUCCESSES: %d", inputTopic.data(), successes);
            }
        }
    }
    if(successes == maxIter){

        cvFindExtrinsicCameraParams2(object_points, image_points,	intrinsic_matrix, distortion_coeffs, rvec, tvec, 0);

        CvMat* rotMat = cvCreateMat(3,3,CV_32FC1);
        CvMat* transMat = cvCreateMat(3,1,CV_32FC1);
        cvRodrigues2(rvec, rotMat, NULL);
        cvTranspose(rotMat, rotMat);
        cvGEMM(rotMat,tvec,-1,NULL,0,transMat,0);

        qDebug("Result of %s", inputTopic.data());
        qDebug("Rotation matrix");
        for(int i=0;i<3;i++){
            qDebug("%f %f %f", CV_MAT_ELEM(*rotMat, float, i, 0), CV_MAT_ELEM(*rotMat, float, i, 1), CV_MAT_ELEM(*rotMat, float, i, 2));
        }
        qDebug("Translation matrix");
        qDebug("%f %f %f", CV_MAT_ELEM(*transMat, float, 0, 0), CV_MAT_ELEM(*transMat, float, 1, 0), CV_MAT_ELEM(*transMat, float, 2, 0));

        cnt = 0;
        isStarted = 0;
        successes = 0;

        emit emitTF(rotMat, transMat);

    }
    emit emitIRImage(IRImage);
}

void CKinectCalibration::run()
{
    ros::start();
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(inputTopic, 1000, &CKinectCalibration::image_cb, this);

    qDebug("Subscribe topic : %s",inputTopic.data());

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    exec();
}

void CKinectCalibration::startCalibration()
{
    isStarted = 1;
}
