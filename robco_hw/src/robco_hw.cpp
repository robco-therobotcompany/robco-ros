#include "ros/ros.h"
#include <robco_hw/robco_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <curl/curl.h>
#include <stdlib.h>
#include <array>
#include <curl/curl.h>
#include <math.h>
#include <sstream>

RobcoHW::RobcoHW(robcomm::Robot& robot) : robot(robot) { 
    num_joints = robot.get_joint_count();

    // Initialize state vectors
    q.assign(num_joints, 0);
    dq.assign(num_joints, 0);
    eff.assign(num_joints, 0);
    cmd.assign(num_joints, 0);

    // Initialize hardware interfaces
    auto q_it = q.begin();
    auto dq_it = dq.begin();
    auto eff_it = eff.begin();
    auto cmd_it = cmd.begin();
    for (int i = 0; i < num_joints; i ++) {
        // Connect and register the joint state interface
        std::ostringstream jointName;
        jointName << "drive" << i << "_joint";
        hardware_interface::JointStateHandle state_handle(jointName.str(), &(*q_it), &(*dq_it), &(*eff_it));
        jnt_state_interface.registerHandle(state_handle);

        // Connect and register the joint position interface
        //hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
        //jnt_pos_interface.registerHandle(pos_handle);

        // Connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(jointName.str()), &(*cmd_it));
        jnt_vel_interface.registerHandle(vel_handle);

        q_it++;
        dq_it++;
        eff_it++;
        cmd_it++;
    }

    //registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);
    registerInterface(&jnt_state_interface);
}

RobcoHW::~RobcoHW() {
}

void RobcoHW::read(std::vector<double> q) {
    if (q.size() != num_joints) {
        printf("error: RobcoHW::read received position vector of wrong size");
        return;
    }

    auto it = this->q.begin();
    for (int i = 0; i <  num_joints; i ++) {
        *(it++) = q[i];
    }
}

void RobcoHW::write() {
    robot.jog_joints(cmd);
}

