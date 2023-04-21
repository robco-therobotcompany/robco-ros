#include "ros/ros.h"
#include <robco_hw/robco_hw.h>
#include <robco_hw/RobcoJointAngles.h>
#include "controller_manager/controller_manager.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <math.h>
#include <robcomm/robcomm.hpp>
#include <std_msgs/Float64.h>

robcomm::Robot robot;
RobcoHW* hw;
controller_manager::ControllerManager* cm;
int jointMsgId = 0;

void updateCallback(const ros::TimerEvent& evt) {
    ros::Duration elapsed_time = ros::Duration(evt.current_real - evt.last_real);

    try {
        robot.receive();
    } catch (const std::runtime_error& e) {
        //printf("Error during receive(): %s", e.what());
    }

    if (hw != nullptr) {
        hw->read(robot.getJointAngles());
    }


    if (cm != nullptr) {
        cm->update(ros::Time::now(), elapsed_time);
    }

    if (hw != nullptr)
        hw->write();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_hw_interface_node");
    ros::NodeHandle n;

    robot.connect("192.168.3.1", 25001, 25000);
    
    // Run update callback at 100Hz, as recommended by Robco UDP documentation
    ros::Timer updateTimer = n.createTimer(ros::Duration(0.01), updateCallback);

    // Wait for robot connection to become initialized
    printf("Waiting for robot to initialize...\n");
    for (int i = 0; i < 1000 && !robot.is_initialized(); i ++) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (!robot.is_initialized()) {
        printf("Timeout waiting for robot to initialize\n");
        return -1;
    }

    hw = new RobcoHW(robot);
    cm = new controller_manager::ControllerManager(hw);

    printf("%d joints found\n", robot.get_joint_count());
    printf("Robco robot interface initialized.\n");

    robot.set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    robot.set_state(robcomm::ROBOT_STATE_CMD_SWITCHED_ON);

    printf("Robco robot interface is exiting.\n");

    return 0;
}
