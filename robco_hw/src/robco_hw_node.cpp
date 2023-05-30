#include <ros/ros.h>
#include <ros/console.h>
#include <robco_hw/robco_hw.h>
#include "controller_manager/controller_manager.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <string>
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

    std::string ipString;
    n.param<std::string>("robot_ip", ipString, "192.168.3.1");

    int localRxPort;
    n.param("local_rx_port", localRxPort, 25001);

    int remoteTxPort;
    n.param("remote_tx_port", remoteTxPort, 25000);

    int initTimeout;
    n.param("robot_init_timeout_seconds", initTimeout, 10);

    ROS_INFO("Attempting to connect to robot at %s (rx %d tx %d)...",
            ipString.c_str(), localRxPort, remoteTxPort);

    robot.connect(ipString, localRxPort, remoteTxPort);
    
    // Run update callback at 100Hz, as recommended by Robco UDP documentation
    ros::Timer updateTimer = n.createTimer(ros::Duration(0.01), updateCallback);

    // Wait for robot connection to become initialized
    ROS_INFO("Waiting for robot to initialize (timeout is %d seconds)...", initTimeout);
    for (int i = 0; i < 100 * initTimeout && !robot.is_initialized(); i ++) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (!robot.is_initialized()) {
        ROS_ERROR("Timeout waiting for robot to initialize\n");
        return -1;
    }

    hw = new RobcoHW(robot);
    cm = new controller_manager::ControllerManager(hw);

    ROS_INFO("%d joints found", robot.get_joint_count());
    ROS_INFO("Robco robot interface initialized.");

    robot.set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    robot.set_state(robcomm::ROBOT_STATE_CMD_SWITCHED_ON);

    ROS_INFO("Robco robot interface is exiting.");

    return 0;
}
