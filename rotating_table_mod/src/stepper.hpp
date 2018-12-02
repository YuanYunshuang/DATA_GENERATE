#ifndef STEPPER_HPP
#define STEPPER_HPP

#define COIL_1A_PIN     14
#define COIL_1B_PIN     15
#define COIL_2A_PIN     17
#define COIL_2B_PIN     18

#define STEPS_PER_ROTATION  200

#define ACCELERATION 120
#define MAX_SPEED 540
#define MIN_SPEED 10


#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/time_duration.hpp>
#include <boost/chrono.hpp>


#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <pigpio.h>


class stepper
{
public:
	stepper();
    ~stepper();

    void startLoop();
    void killLoop();
    void setVelocity(float TargetVelocity);
    void setVelocityWithRamp(float TargetVelocity);
    void setVelocityWithRampThreaded(float TargetVelocity);
    void setPosition(float TargetPosition);
    void setPositionThreaded(float TargetPosition);
    void getPosition(float &CurrentPosition);
    void getVelocity(float &CurrentVelocity);


private:

    void LoopThreaded();
    boost::thread ControlThread;
    boost::thread PositionModeThread;
    boost::mutex m_DelayMutex;
    boost::mutex m_ClockwiseBoolMutex;
    boost::mutex m_StopBoolMutex;
    boost::mutex m_PositionStepsMutex;
    boost::mutex m_ThreadActiveBoolMutex;
    bool m_bPostitionThreadActive;

    int m_nDelay; // Zeit zwischen den Schrittwechsel in Mikrosekunden
    float m_fVelocity; //Winkelgeschindigkeit in GradProSekunde

    int m_nStep;
    bool m_bClockwise; // Rotationsrichtung Urzeigersinn=1; GegenUhrzeigersinn=0;
    bool m_bStop;

    int m_nCurrentPositionSteps;




};


#endif // STEPPER_HPP
