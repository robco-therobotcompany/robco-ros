#ifndef ROBCO_HW_H
#define ROBCO_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <array>
#include <curl/curl.h>
#include <robcomm/robcomm.hpp>
#include <list>

const int ROBCO_MAX_JOINTS = 12; // Currently, a maximum of 12 joints are supported

class RobcoHW : public hardware_interface::RobotHW
{
    public:
        /**
         * @brief Initialize a new hardware interface.
         * 
         * @param robot Instance of librobcomm robot object to use for communication.
         */
        RobcoHW(robcomm::Robot& robot);
        ~RobcoHW();

        /**
         * @brief Update robot model with given joint angles.
         * 
         * @param q New joint angles
         */
        void read(std::vector<double> q);

        /**
         * @brief Send currently commanded values to robot. To be called cyclically.
         */
        void write();

        /**
         * @brief Returns the current command values as a vector.
         * 
         * @return Vector of currently commanded values
         */
        std::vector<double> getCmdVector();

    private:
        robcomm::Robot& robot;

        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;

        int num_joints;
        std::list<double> cmd;
        std::list<double> q;
        std::list<double> dq;
        std::list<double> eff;
};

#endif // ROBCO_HW_H

