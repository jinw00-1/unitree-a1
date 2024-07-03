#ifndef BALANCE_MOTION_H
#define BALANCE_MOTION_H

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"

namespace unitree_model {
    extern ros::Publisher servo_pub[12];
    extern unitree_legged_msgs::LowCmd lowCmd;
    extern unitree_legged_msgs::LowState lowState;

    void balance_motion_init(double pitch);
    void paramInit();
    void moveAllPosition(double* targetPos, double duration);
    void sendServoCmd();
}


#endif // BALANCE_MOTION_H