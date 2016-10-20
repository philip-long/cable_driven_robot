// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>


#include <br_driver/controller_class.h>



// Function to load attachment points


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(1);
    int number_of_cables;

    nh.getParam("number_of_cables",number_of_cables);

    controller_class CableRobot(nh,number_of_cables);

    sensor_msgs::JointState joint_state_out,joint_deviation;
    joint_deviation.name.push_back("q1");
    joint_deviation.name.push_back("q4");
    joint_deviation.name.push_back("q2");
    joint_deviation.name.push_back("q5");
    joint_deviation.name.push_back("q3");
    joint_deviation.name.push_back("q6");
    joint_deviation.name.push_back("q7");
    joint_deviation.name.push_back("q8");

    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.0);
    ros::Publisher joint_deviation_publisher=
            nh.advertise<sensor_msgs::JointState>("joint_deviation",1);

    //Transform of attachment points w.r.t to platform centre in platform frame
    vpHomogeneousMatrix wTp;

    double ratio=9/3.1428;  // 360/Pi/Diameter

    while (ros::ok()) {
        CableRobot.GetPlatformTransformation(wTp);
        CableRobot.printfM((wTp));
        ros::spinOnce();
    }




    return 0;

}

