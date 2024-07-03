#include "balance_motion.h"
#include "body.h"

namespace unitree_model {

    ros::Publisher servo_pub[12];
    unitree_legged_msgs::LowCmd lowCmd;
    unitree_legged_msgs::LowState lowState;

    void balance_controller(double pitch)
    {
        while(ros::ok())
            {double pos[12] = {0.0, 0.67-pitch, -1.3, -0.0, 0.67-pitch, -1.3, 
                        0.0, 0.67-pitch, -1.3, -0.0, 0.67-pitch, -1.3};
            moveAllPosition(pos, 2*1000);
            }
    }


    void balance_motion_init(double pitch) {
        paramInit();
        balance_controller(pitch);
    }

}