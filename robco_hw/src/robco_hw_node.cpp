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
    ros::Duration elapsed_time = ros::Duration(evt.current_real - evt.last_real);

    robot->receive();
    hw->read(robot->getJointAngles());

    cm->update(ros::Time::now(), elapsed_time);

    hw->write();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_hw_interface_node");
    ros::NodeHandle n;

    robot = new robcomm::Robot("192.168.178.93", 25002, 25000);
    hw = new RobcoHW(*robot);
    cm = new controller_manager::ControllerManager(hw);

    // Run update callback at 100Hz, as recommended by Robco UDP documentation
    ros::Timer updateTimer = n.createTimer(ros::Duration(0.01), updateCallback);

    printf("Robco robot interface initialized.\n");
    
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    printf("Robco robot interface is exiting.\n");

    return 0;
}
