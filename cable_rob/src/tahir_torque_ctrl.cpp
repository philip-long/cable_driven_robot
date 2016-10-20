// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>

#include <cable_rob/CartesianTrajectory.h>
#include <cable_rob/CartesianTrajectoryPoint.h>

#include <cable_rob/controller_class.h>


// Function to load attachment points


int main(int argc, char **argv) {
    ros::init(argc, argv, "CartesianController");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    int number_of_cables;

    double ratio;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);

    controller_class CableRobot(nh,number_of_cables,"~",false); // initialise class without publisher
    ros::Publisher joint_deviation_publisher=
            nh.advertise<sensor_msgs::JointState>("desired_joint_position",1);

    sensor_msgs::JointState desired_joint_position;

    desired_joint_position.name.push_back("q1");
    desired_joint_position.name.push_back("q2");
    desired_joint_position.name.push_back("q3");
    desired_joint_position.name.push_back("q4");
    desired_joint_position.name.push_back("q5");
    desired_joint_position.name.push_back("q6");
    desired_joint_position.name.push_back("q7");
    desired_joint_position.name.push_back("q8");

    desired_joint_position.position.resize(number_of_cables,0);
    desired_joint_position.velocity.resize(number_of_cables,0);
    desired_joint_position.effort.resize(number_of_cables,0);


    //    while(!CableRobot.GetJointFlag())
    //    {
    //        CableRobot.GetRobotJointState(desired_joint_position);
    //        ROS_INFO("Got_flag");
    //        ros::spinOnce();
    //        r.sleep();
    //    }



    std::vector<double> tau1;
    std::vector<double> tau2;
    std::vector<double> tau3;
    std::vector<double> tau4;
    std::vector<double> tau;
    std::vector<double> p_x;
    std::vector<double> p_y;
    std::vector<double> p_z;
    std::vector<double> traj_time;
    //std::vector<double> Current;

    tau1=CableRobot.get_trajectory_parameter("tension1");ROS_INFO("1");
    tau2=CableRobot.get_trajectory_parameter("tension2");ROS_INFO("1");
    tau3=CableRobot.get_trajectory_parameter("tension3");ROS_INFO("1");
    tau4=CableRobot.get_trajectory_parameter("tension4");ROS_INFO("1");
    p_x=CableRobot.get_trajectory_parameter("position_x");ROS_INFO("1");
    p_y=CableRobot.get_trajectory_parameter("position_y");ROS_INFO("1");
    p_z=CableRobot.get_trajectory_parameter("position_z");ROS_INFO("1");
    traj_time=CableRobot.get_trajectory_parameter("timeee");ROS_INFO("1");


    std::cout<<desired_joint_position.position.size()<<desired_joint_position.effort.size()<<std::endl;


    ros::Time Start= ros::Time::now();
    ros::Duration Current;
    double torque;
    ros::Duration(2.0).sleep();
    while(ros::ok())
    {
        //ROS_INFO("Tau=[ %f, %f,%f,%f], Time=%f, Position=[ %f, %f,%f]",tau1[i],tau2[i],tau3[i],tau4[i],traj_time[i],p_x[i],p_y[i],p_z[i]);

        Current=(ros::Time::now())-Start;

        torque=0.0;

        for (int time_point = 0; time_point < traj_time.size(); ++time_point)
        {
            ROS_INFO("Current time=%f  traj_time[i]=%f",Current.toSec(),traj_time[time_point]);
            if (Current.toSec()<traj_time[time_point])
            {
                torque=tau1[time_point-1];
                break;
            }
        }
          desired_joint_position.header.stamp=ros::Time::now();
          desired_joint_position.position[0]=0.5;
          desired_joint_position.effort[0]=-0.002*2*torque;
//        desired_joint_position.effort[1]=tau2[i];
//        desired_joint_position.effort[2]=tau3[i];
//        desired_joint_position.effort[3]=tau4[i];


        joint_deviation_publisher.publish(desired_joint_position);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}



