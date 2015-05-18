#ifndef CKINECTTHREAD_H
#define CKINECTTHREAD_H

#include <ros/ros.h>
#include <QThread>

class CKinectThread : public QThread
{
    Q_OBJECT
public:
    CKinectThread();

public:
    void run();

protected:

    virtual void init()=0;


};

#endif // CKINECTTHREAD_H
