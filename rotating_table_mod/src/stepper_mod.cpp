#include "steppermodified.hpp"




stepperdir::stepperdir()
{
    m_nStep=1;
    m_fVelocity=0;
    m_nDelay=100000;
    m_bClockwise=true;
    m_bStop=true;
    m_nCurrentPositionSteps=0;
    m_bPostitionThreadActive=false;

    //Hier PiGPIO initialisieren

    //Initialize pigpio for motor control
    if (gpioInitialise() < 0)
    {
       fprintf(stderr, "pigpio initialisation failed\n");
       //return 1;
    }

    // Set GPIO modes
    gpioSetMode(COIL_1A_PIN, PI_OUTPUT);
    gpioSetMode(COIL_1B_PIN, PI_OUTPUT);
    gpioSetMode(COIL_2A_PIN, PI_OUTPUT);
    gpioSetMode(COIL_2B_PIN, PI_OUTPUT);

}

stepperdir::~stepperdir()
{

}


void stepperdir ::setVelocity(float TargetVelocity)
{
    if(TargetVelocity<0)
    {
        TargetVelocity=-TargetVelocity;
        m_ClockwiseBoolMutex.lock();
        m_bClockwise=false;
        m_ClockwiseBoolMutex.unlock();
    } else
    {
        m_ClockwiseBoolMutex.lock();
        m_bClockwise=true;
        m_ClockwiseBoolMutex.unlock();
    }

    m_fVelocity=TargetVelocity;
    if(abs(m_fVelocity)>MIN_SPEED)
    {
    m_DelayMutex.lock();
    m_nDelay=(360*1000000)/(STEPS_PER_ROTATION*m_fVelocity);
    m_DelayMutex.unlock();
    m_StopBoolMutex.lock();
    m_bStop=false;
    m_StopBoolMutex.unlock();

    } else
    {
        //std::cout << "ERROR: Durchs NULL teilen ist unmöglich" << std::endl;
        std::cout << "MOTOR ANGEHALTEN" << std::endl;
        m_StopBoolMutex.lock();
        m_bStop=true;
        m_StopBoolMutex.unlock();
    }
}

void stepperdir::setVelocityWithRamp(float TargetVelocity)
{
    float CurrentVelocity=0.0;
    float Aufloesung=10;
    m_DelayMutex.lock();
    CurrentVelocity=(360*1000000)/(m_nDelay*STEPS_PER_ROTATION);
    m_DelayMutex.unlock();
    bool Direction=false;
    m_ClockwiseBoolMutex.lock();
    Direction=m_bClockwise;
    m_ClockwiseBoolMutex.unlock();
    if(!Direction)
    {
        CurrentVelocity=-CurrentVelocity;
    }

    while(abs(TargetVelocity-CurrentVelocity)>ACCELERATION/2.0/Aufloesung)
    {
        if((TargetVelocity-CurrentVelocity)>0)
        {
            //Beschleunigungsrampe hier
            CurrentVelocity=CurrentVelocity+ACCELERATION/Aufloesung;
            this->setVelocity(CurrentVelocity);

        }
        else
        {
            //Verzögerungsrampe hier
            CurrentVelocity=CurrentVelocity-ACCELERATION/Aufloesung;
            this->setVelocity(CurrentVelocity);
        }
        usleep(1000000/Aufloesung);
    }
    this->setVelocity(TargetVelocity);
    usleep(1000000/Aufloesung);
    std::cout << "Rampe fertig, Geschwindigkeit " << CurrentVelocity << " erreicht" << std::endl;
    m_ThreadActiveBoolMutex.lock();
    m_bPostitionThreadActive=false;
    m_ThreadActiveBoolMutex.unlock();
}

void stepperdir::setVelocityWithRampThreaded(float TargetVelocity)
{
    bool ThreadActive=true;
    m_ThreadActiveBoolMutex.lock();
    ThreadActive=m_bPostitionThreadActive;
    m_ThreadActiveBoolMutex.unlock();
    if(ThreadActive)
    {
//        if(PositionModeThread.try_join_for(boost::chrono::milliseconds(1)))
//        {
//            PositionModeThread= boost::thread(&stepper::setVelocityWithRamp,this,TargetVelocity);
//        }else
//        {
//            std::cout << "Sorry position or velocity ramp generator did not finished yet,";
//            std::cout << "this command will not be executed" << std::endl;
//        }
        std::cout << "Sorry position or velocity ramp generator did not finished yet,";
        std::cout << "this command will not be executed" << std::endl;
    }else
    {
        m_ThreadActiveBoolMutex.lock();
        m_bPostitionThreadActive=true;
        m_ThreadActiveBoolMutex.unlock();
        PositionModeThread= boost::thread(&stepperdir::setVelocityWithRamp,this,TargetVelocity);
    }
}

void stepperdir::setPositionWithDir(float TargetPosition, float Richtung)
{
    int TargetPositionInSteps=TargetPosition*200/360;


    //Deklarationen und Definitionen für den Ramp-Generator
    float P_s;
    float P_z=TargetPosition;
    float V_max_grenze=MAX_SPEED;
    float A_max=ACCELERATION;
    float time_step=0.1;

    float V_max=0;
    float t1=0;
    float t2=0;

    this->getPosition(P_s);
    std::cout << "P_s" << P_s << std::endl;

    // Beschleunigte Bewegung
    // S=a/2*t^2+v0*t+s0
    // V=a*t+v0

    // Berechnen des Zeitbedarfes

    // t1=V_max/A_max;  Zeit von Anfang bis zum Ende der  Beschleunigungsphase
    // t2=S/V_max;  Zeit von Anfang der Bewegung bis zum Anfang der Bremsphase

    float Smin=V_max_grenze*V_max_grenze/A_max;
    Smin = Smin / 360*200;
    float S;
    //float S=abs(P_z-P_s); // Betrag der Strecke
    float dir = (P_z-P_s)/abs(P_z-P_s);
    if ((dir < 0 && Richtung < 0) || (dir>0 && Richtung >0)){
        float S1=abs(P_z-P_s);
        S=S1;
        std::cout << "Fall 1.1" << std::endl;
    }else{
        float S1 = FULL_CIRCLE - abs(P_z - P_s);
std::cout << "S= " << abs(P_z - P_s) << std::endl;
        S=S1;
        std::cout << "Fall 1.2" << std::endl;
    }
    //float Richtung=(P_z-P_s)/abs(P_z-P_s); // Drehrichtung
    if(S>=Smin)
    {
        std::cout << "S= " << S << "  Smin= " << Smin << std::endl;
        V_max=V_max_grenze;
        t1=V_max/A_max; //Zeit von Anfang bis zum Ende der  Beschleunigungsphase
        t2=S/V_max; // Zeit von Anfang der Bewegung bis zum Anfang der Bremsphase
        std::cout << "Fall 2.1" << std::endl;
    } else if(S<Smin)
    {
        std::cout << "S= " << S << "  Smin= " << Smin << std::endl;
        V_max=sqrt(abs(S)*A_max);
        t1=V_max/A_max;
        t2=t1;
        std::cout << "Fall 2.2" << std::endl;
    }

    float t_gesamt=t1+t2;

    std::cout << "T1= " << t1 << "  T2= " << t2 << std::endl;



    bool Ready=false;
    int state=0;
    float t=0.0;
    float S0=0.0;
    float V0=0.0;
    float A=0.0;
    std::vector<float> S1;
    std::vector<float> V1;
    std::vector<float> S2;
    std::vector<float> V2;
    std::vector<float> S3;
    std::vector<float> V3;

    //mit boost
    boost::posix_time::ptime t_start, t_last;
    boost::posix_time::time_duration t_duration;
    t_start=boost::posix_time::microsec_clock::local_time();
    t_last=boost::posix_time::microsec_clock::local_time();


    while(!Ready)
    {
        t_duration=boost::posix_time::microsec_clock::local_time()-t_start;
//        std::cout << "T= " << t_duration.total_microseconds() << std::endl;
        t=t_duration.total_milliseconds()/1000.0;
//        std::cout << "T= " << t << std::endl;
        switch (state) {
        case 0:
            state=1;
            break;
        case 1:

            //// Beschleunigungsphase

//            std::cout << "Beschleunigungsphase" << std::endl;

            S0=P_s;
            V0=0; //Hier Anfangsgeschwindigkeit angeben
            A=A_max*Richtung;

            // t=[0:time_step:time_end1];

            // S=a/2*t^2+v0*t+s0
            // S=P_s +V0*t+A/2*t^2;
//            std::vector<float> S1;//S1=polyval([A/2 V0 S0],t);
//            std::vector<float> V1;//V1=polyval([A V0],t);

            if( t<=t1)
            {
//                t=t+time_step;
                S1.push_back(A/2.0*t*t + V0*t + S0);
                V1.push_back(A*t + V0);
                this->setVelocity(V1.back());
//                std::cout << "Geschwindigkeit=" << V1.back() << std::endl;
            } else
            {
                state=2;
                std::cout << "Beschleunigung zu ende" << std::endl;
            }


            break;
        case 2:
            //// unbeschleunigte Bewegung

//            std::cout << "unbeschleunigte Bewegung" << std::endl;
            if(S1.size()>0)
            {
                S0=S1.back();
                V0=V1.back();
            }else
            {
                S0=0;
                V0=0;
            }

            A=0;

            // time_end2=hängt vom Zeitbedarf für die gesammte bewegung ab

            //t=[time_step:time_step:time_end2];
//            std::vector<float> S2;//S2=polyval([A/2 V0 S0],t);
//            std::vector<float> V2;//V2=polyval([A V0],t);

            if(t<=t2)
            {
//                std::cout << "t-t1=" << t-t1 << std::endl;
//                t=t+time_step;
                S2.push_back(A/2.0*(t-t1)*(t-t1) + V0*(t-t1) + S0);
                V2.push_back(A*(t-t1) + V0);
                this->setVelocity(V2.back());
//                std::cout << "Geschwindigkeit=" << V2.back() << std::endl;
            }else
            {
                state=3;
                std::cout << "Bremsphase" << std::endl;
            }

            break;
        case 3:
            /// Bremsphase

//            std::cout << "Bremsphase" << std::endl;

            if(S2.size()>0)
            {
                S0=S2.back();
                V0=V2.back();
            } else if(S1.size()>0)
            {
                S0=S1.back();
                V0=V1.back();
            }else
            {
                S0=0;
                V0=0;
            }
            A=-A_max*Richtung;

            //t=[time_step:time_step:time_end3];
            //std::vector<float> S3;//S3=polyval([A/2 V0 S0],t);
            //std::vector<float> V3;//V3=polyval([A V0],t);

            if(t<=(t2+t1))
            {
//                t=t+time_step;
                S3.push_back(A/2.0*(t-t2)*(t-t2) + V0*(t-t2) + S0);
                V3.push_back(A*(t-t2) + V0);
                this->setVelocity(V3.back());
//                std::cout << "Geschwindigkeit=" << V3.back() << std::endl;
            }else
            {
                state=4;
                std::cout << "Nachkorigieren" << std::endl;
            }
            break;
        case 4:
            // Position abruffen (IST und SOLL) und mit kleiner Geschwindigkeit nachkorrigieren, bis die übereinstimmen
            int PositionStepNow=0;
            m_PositionStepsMutex.lock();
            PositionStepNow=m_nCurrentPositionSteps;
            m_PositionStepsMutex.unlock();
            int diff=TargetPositionInSteps-PositionStepNow;
//            if(diff!=0)
//            {
//                this->setVelocity(diff/abs(diff)*360.0/STEPS_PER_ROTATION/time_step);
//            }else
            {
                Ready=true;
                std::cout << "Position Erreicht" << std::endl;
                this->setVelocity(0.0);
            }

            break;
//        default:
//            break;
        }

        //Hier die Zeitberechnung
//        float DauerMicrosec=clock()-t_last;
//        usleep(time_step*1000000.0-DauerMicrosec);
//        t_last=clock();
        boost::posix_time::time_duration DauerMicrosec=boost::posix_time::microsec_clock::local_time()-t_last;
        usleep(time_step*1000000.0-DauerMicrosec.total_microseconds());
        t_last=boost::posix_time::microsec_clock::local_time();

    }
    m_ThreadActiveBoolMutex.lock();
    m_bPostitionThreadActive=false;
    m_ThreadActiveBoolMutex.unlock();
}

void stepperdir::setPositionThreadedWithDir(float TargetPosition,float Richtung)
{
    bool MotorHold=false;
    m_StopBoolMutex.lock();
    MotorHold=m_bStop;
    m_StopBoolMutex.unlock();
    bool ThreadActive=true;
    m_ThreadActiveBoolMutex.lock();
    ThreadActive=m_bPostitionThreadActive;
    m_ThreadActiveBoolMutex.unlock();
    if(MotorHold)
    {
        if(ThreadActive)
        {
//            if(PositionModeThread.try_join_for(boost::chrono::milliseconds(1)))
//            {
//                PositionModeThread= boost::thread(&stepper::setPosition,this,TargetPosition);
//            }else
//            {
//                std::cout << "Sorry position or velocity ramp generator did not finished yet,";
//                std::cout << "this command will not be executed" << std::endl;
//            }
            std::cout << "Sorry position or velocity ramp generator did not finished yet,";
            std::cout << "this command will not be executed" << std::endl;

        }else
        {
            m_ThreadActiveBoolMutex.lock();
            m_bPostitionThreadActive=true;
            m_ThreadActiveBoolMutex.unlock();
            PositionModeThread= boost::thread(&stepperdir::setPositionWithDir,this,TargetPosition, Richtung);
        }
    }else
    {
        this->setVelocityWithRampThreaded(0.0);
        std::cout << "Sorry, the Speed is not NULL. it will be set to null but this command will not be executed." << std::endl;
        std::cout << "Please try once more!" << std::endl;
    }


}

void stepperdir::getPosition(float &CurrentPosition)
{
    float PositionSteps=0;
    m_PositionStepsMutex.lock();
    PositionSteps=m_nCurrentPositionSteps;
    m_PositionStepsMutex.unlock();
    CurrentPosition=PositionSteps/200*360;
//    std::cout <<"CurrentPos= " << CurrentPosition << std::endl;
}

void stepperdir::getVelocity(float &CurrentVelocity)
{
    float currentDelay;
    m_DelayMutex.lock();
    currentDelay=m_nDelay;
    m_DelayMutex.unlock();
    bool Direction=0;
    m_ClockwiseBoolMutex.lock();
    Direction=m_bClockwise;
    m_ClockwiseBoolMutex.unlock();

    CurrentVelocity=360*1000000/(STEPS_PER_ROTATION*currentDelay);
    if(!Direction)
    {
        CurrentVelocity=-CurrentVelocity;
    }
    bool stop=false;
    m_StopBoolMutex.lock();
    stop=m_bStop;
    m_StopBoolMutex.unlock();
    if(stop==true)
    {
        CurrentVelocity=0.0;
    }
}


//void stepper::setPosition(float TargetPosition)
//{
//    /*Hier muss eine Rampe erzeugt werde
//     * Formeln:
//     *
//     * s=a/2*t^2 +v0*t+s0
//     * v=a*t+v0
//     *
//     *
//     *
//     *
//     */
//    int acc=ACCELERATION;
//    int Vmax=720;
//    int MaxBreakingWay=Vmax*Vmax/2/acc;
//    //Weg berechnen

//    int CurrentStep=0;
//    m_PositionStepsMutex.lock();
//    CurrentStep=m_nCurrentPositionSteps;
//    m_PositionStepsMutex.unlock();

//    float CurrentPosition=CurrentStep/STEPS_PER_ROTATION*360;
//    float Differense=TargetPosition-CurrentPosition;
//    bool Ready=false;
//    while(!Ready)
//    {
//        float CurrentVelocity=0.0;
//        int delay;
//        m_DelayMutex.lock();
//        delay=m_nDelay;
//        m_DelayMutex.unlock();
//        CurrentVelocity=(360*1000000)/(delay*STEPS_PER_ROTATION);

//        float BreakingWay=CurrentVelocity*CurrentVelocity/2/acc;

//        if(abs(Differense)<abs(BreakingWay))
//        {
//            //bremsen
//            this->setVelocityWithRamp(0.0);
//            Ready=true;

//        }else
//        {
//            //Beschleunigen auf maximalgeschwindigkeit

//            float Aufloesung=10;

//            if(abs(Vmax-CurrentVelocity)>ACCELERATION/2.0/Aufloesung)
//            {
//                if((Vmax-CurrentVelocity)>0)
//                {
//                    //Beschleunigungsrampe hier
//                    CurrentVelocity=CurrentVelocity+ACCELERATION/Aufloesung;
//                    this->setVelocity(CurrentVelocity);

//                }
//                else
//                {
//                    //Verzögerungsrampe hier
//                    CurrentVelocity=CurrentVelocity-ACCELERATION/Aufloesung;
//                    this->setVelocity(CurrentVelocity);
//                }
//                usleep(1000000/Aufloesung);
//            }else
//            {
//                std::cout << "Rampe fertig, Geschwindigkeit " << CurrentVelocity << " erreicht" << std::endl;
//                //Ready=true;
//            }

//        }

//    }
//    std::cout << "Position reached" << std::endl;
//}

void stepperdir::startLoop()
{
    ControlThread= boost::thread(&stepperdir::LoopThreaded,this);
}

void stepperdir::LoopThreaded()
{
    while(1)
    {
        if(m_nStep==1)
        {
            // hier 1A=PLUS
            //      1B=MINUS
            //      2A=PLUS
            //      2B=MINUS

            gpioWrite(COIL_1A_PIN, 1);
            gpioWrite(COIL_1B_PIN, 0);
            gpioWrite(COIL_2A_PIN, 1);
            gpioWrite(COIL_2B_PIN, 0);

        }
        if(m_nStep==2)
        {
            // hier 1A=PLUS
            //      1B=MINUS
            //      2A=MINUS
            //      2B=PLUS
            gpioWrite(COIL_1A_PIN, 1);
            gpioWrite(COIL_1B_PIN, 0);
            gpioWrite(COIL_2A_PIN, 0);
            gpioWrite(COIL_2B_PIN, 1);

        }
        if(m_nStep==3)
        {
            // hier 1A=MINUS
            //      1B=PLUS
            //      2A=MINUS
            //      2B=PLUS
            gpioWrite(COIL_1A_PIN, 0);
            gpioWrite(COIL_1B_PIN, 1);
            gpioWrite(COIL_2A_PIN, 0);
            gpioWrite(COIL_2B_PIN, 1);

        }
        if(m_nStep==4)
        {
            // hier 1A=MINUS
            //      1B=PLUS
            //      2A=PLUS
            //      2B=MINUS
            gpioWrite(COIL_1A_PIN, 0);
            gpioWrite(COIL_1B_PIN, 1);
            gpioWrite(COIL_2A_PIN, 1);
            gpioWrite(COIL_2B_PIN, 0);

        }

        // Rotaitonsrichtung

        bool temproraryClockwise=true;
        m_ClockwiseBoolMutex.lock();
        temproraryClockwise=m_bClockwise;
        m_ClockwiseBoolMutex.unlock();


        bool bTempStop=true;
        m_StopBoolMutex.lock();
        bTempStop=m_bStop;
        m_StopBoolMutex.unlock();



        if(temproraryClockwise && !bTempStop)
        {
                m_nStep++;
                m_PositionStepsMutex.lock();
                m_nCurrentPositionSteps++;
                m_PositionStepsMutex.unlock();
        } else if(!bTempStop)
        {
            m_nStep--;
            m_PositionStepsMutex.lock();
            m_nCurrentPositionSteps--;
            m_PositionStepsMutex.unlock();
        }

        if(m_nStep>4)
        {
            m_nStep=1;
        }
        if(m_nStep<1)
        {
            m_nStep=4;
        }



        int temproraryDelay=0;
        m_DelayMutex.lock();
        temproraryDelay=m_nDelay;
        m_DelayMutex.unlock();
        usleep(temproraryDelay);
    }


}
