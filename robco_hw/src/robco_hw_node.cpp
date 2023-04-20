#include "ros/ros.h"
#include <robco_hw/robco_hw.h>
#include <robco_hw/RobcoJointAngles.h>
#include "controller_manager/controller_manager.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <math.h>
#include <robcomm/robcomm.hpp>

robcomm::Robot* robot;
RobcoHW* hw;
controller_manager::ControllerManager* cm;
ros::Subscriber websocketSub;
//ros::Publisher jointPub;
int jointMsgId = 0;

void updateCallback(const ros::TimerEvent& evt) {
    //ros::Duration elapsed_time = ros::Duration(evt.current_real - evt.last_real);

    robot->receive();
    if (hw != nullptr)
        hw->read(robot->getJointAngles());

    if (cm != nullptr) {
        //cm->update(ros::Time::now(), elapsed_time);
        cm->update(ros::Time::now(), ros::Duration(0.01));
    }

    if (hw != nullptr)
        hw->write();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_hw_interface_node");
    ros::NodeHandle n;

    robot = new robcomm::Robot("192.168.178.93", 25001, 25000);
    robot->connect();
    
    // Run update callback at 100Hz, as recommended by Robco UDP documentation
    ros::Timer updateTimer = n.createTimer(ros::Duration(0.01), updateCallback);

    // Wait for first messages
    printf("Waiting for robot to initialize (TODO)\n");
    for (int i = 0; i < 10; i ++) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    hw = new RobcoHW(*robot);
    cm = new controller_manager::ControllerManager(hw);

    printf("Robco robot interface initialized.\n");

    robot->set_state(robcomm::ROBOT_STATE_CMD_OPERATIONAL);
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    //for (int i = 0; i < 1000; i ++) {
    //    ros::TimerEvent evt;
    //    updateCallback(evt);
    //    ros::spinOnce();
    //    ros::Duration(0.01).sleep();
    //}

    robot->set_state(robcomm::ROBOT_STATE_CMD_SWITCHED_ON);

    printf("Robco robot interface is exiting.\n");

    return 0;
}
