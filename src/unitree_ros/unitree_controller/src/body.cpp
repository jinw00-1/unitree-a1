/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"
#include "balance_motion.h"
#include "ros/ros.h"

namespace unitree_model {

    extern ros::Publisher servo_pub[12];
    extern unitree_legged_msgs::LowCmd lowCmd;
    extern unitree_legged_msgs::LowState lowState;

    // These parameters are only for reference.
    // Actual patameters need to be debugged if you want to run on real robot.
    void paramInit()
    {
        for(int i=0; i<4; i++){
            lowCmd.motorCmd[i*3+0].mode = 0x0A;
            lowCmd.motorCmd[i*3+0].Kp = 70;
            lowCmd.motorCmd[i*3+0].dq = 0;
            lowCmd.motorCmd[i*3+0].Kd = 3;
            lowCmd.motorCmd[i*3+0].tau = 0;
            lowCmd.motorCmd[i*3+1].mode = 0x0A;
            lowCmd.motorCmd[i*3+1].Kp = 180;
            lowCmd.motorCmd[i*3+1].dq = 0;
            lowCmd.motorCmd[i*3+1].Kd = 8;
            lowCmd.motorCmd[i*3+1].tau = 0;
            lowCmd.motorCmd[i*3+2].mode = 0x0A;
            lowCmd.motorCmd[i*3+2].Kp = 300;
            lowCmd.motorCmd[i*3+2].dq = 0;
            lowCmd.motorCmd[i*3+2].Kd = 15;
            lowCmd.motorCmd[i*3+2].tau = 0;
        }
        for(int i=0; i<12; i++){
            lowCmd.motorCmd[i].q = lowState.motorState[i].q;
        }
    }

    void stand()
    {   
        double t = 0.0;
        while(ros::ok())
            {double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                        0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
            moveAllPosition(pos, 2*1000);
            t += 1;
            }
    }
    ///////////******////////////
    void half()
    {   
        long long time_=0.0;
        const double period_ = 50000; //ms;
        std::vector<double> final_value;
        std::vector<double> start_point;
        std::vector<double> p0;
        std::vector<double> p1;
        std::vector<double> p2;
        std::vector<double> p3;
        double position1, position2, position3;
        p0 = {0.0, 0.8, -1.3};
        p1 = {0.0, 0.73, -2.0};
        p2 = {0.0, 0.5, -2.0};
        p3 = {0.0, 0.3, -1.3};

        double x0 = p0[0];
        double y0 = p0[1];
        double z0 = p0[2];

        double x1 = p1[0];
        double y1 = p1[1];
        double z1 = p1[2];

        double x2 = p2[0];
        double y2 = p2[1];
        double z2 = p2[2];

        double x3 = p3[0];
        double y3 = p3[1];
        double z3 = p3[2];

            
        for(time_=0.0; time_<=100000; time_++){
            if(time_<=50000){
                position1 = x0*pow((1-(double)time_/period_),3)+3*pow((1-(double)time_/period_),2)*((double)time_/period_)*x1+3*(1-(double)time_/period_)*pow((double)time_/period_,2)*x2+pow((double)time_/period_,3)*x3;
                position2 = y0*pow((1-(double)time_/period_),3)+3*pow((1-(double)time_/period_),2)*((double)time_/period_)*y1+3*(1-(double)time_/period_)*pow((double)time_/period_,2)*y2+pow((double)time_/period_,3)*y3;
                position3 = z0*pow((1-(double)time_/period_),3)+3*pow((1-(double)time_/period_),2)*((double)time_/period_)*z1+3*(1-(double)time_/period_)*pow((double)time_/period_,2)*z2+pow((double)time_/period_,3)*z3;

                double pos_h[12] = {position1, position2, position3, position1, position2, position3, 
                                    position1, position2, position3, position1, position2, position3};
                moveAllPosition(pos_h, 5*1000);
            }else{
                position1 = x3 + (time_/period_-1)*(x1-x3);
                position2 = y3 + (time_/period_-1)*(y1-y3);
                position3 = z3 + (time_/period_-1)*(z1-z3);

                double pos_h[12] = {position1, position2, position3, position1, position2, position3, 
                                    position1, position2, position3, position1, position2, position3};
                moveAllPosition(pos_h, 5*1000);
                }       
            }
    }


    //////////////////////////

    void motion_init()
    {
        paramInit();
        stand();
    }

    //////*******////////
    void half_motion_init()
    {
        paramInit();
        half();
    }


    void sendServoCmd()
    {
        for(int m=0; m<12; m++){
            servo_pub[m].publish(lowCmd.motorCmd[m]);
        }
        ros::spinOnce();
        usleep(1000);
    }



    void moveAllPosition(double* targetPos, double duration)
    {
        double pos[12] ,lastPos[12], percent;
        for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
        for(int i=1; i<=duration; i++){
            if(!ros::ok()) break;
            percent = (double)i/duration;
            for(int j=0; j<12; j++){
                lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
            }
            sendServoCmd();
        }
    }


}