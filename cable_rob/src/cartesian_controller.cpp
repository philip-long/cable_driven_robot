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

    sensor_msgs::JointState current_joint_position,desired_joint_position;
    desired_joint_position.name.push_back("q1");
    desired_joint_position.name.push_back("q2");
    desired_joint_position.name.push_back("q3");
    desired_joint_position.name.push_back("q4");
    desired_joint_position.name.push_back("q5");
    desired_joint_position.name.push_back("q6");
    desired_joint_position.name.push_back("q7");
    desired_joint_position.name.push_back("q8");

    desired_joint_position.position.resize(number_of_cables);
    current_joint_position=desired_joint_position;

    std::vector<double> desired_cable_length(number_of_cables);
    std::vector<double> current_cable_length(number_of_cables);

    ros::Publisher joint_deviation_publisher=
            nh.advertise<sensor_msgs::JointState>("desired_joint_position",1);


    //Transform of attachment points w.r.t to platform centre in platform frame



    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    ros::Time now = ros::Time(0);

    vpHomogeneousMatrix wTp_final,wTp_desired,wTp_estimate,wTp_initial,wTp_desired_last;

    vpColVector Error(6);
    vpColVector dX(6);
    double tol=0.002;
    std::vector<double> dq(number_of_cables);


    // *************************

    // Two loops one outer that waits for desired transform and checks that its is different to other

    // Second loop plots a simply trajectory between the two and sends a desired joint position to the robot
    // Loop exits upon completion or cancel flag
    // Loop details :
    //      Check current joint position
    //      Update transformation using last known and current joint position
    //      Define new desired position
    ros::Time begin = ros::Time::now();
    ros::Duration timefinal(10.0); // total time for slowdown 200ms
    ros::Duration current_time;
    vpMatrix J_lau(8,6); // jacobian matrix
    vpColVector dl_jac(number_of_cables);


    while(ros::ok())
    {
        std::setprecision(7);
        if(CableRobot.GetDesiredTransformFlag()) // I have recieved a desired position
        {
            // Check if the desired position is far away from initial position
            CableRobot.GetDesiredPlatformTransformation(wTp_final); // Get desired Platform
            CableRobot.GetEstimatedPlatformTransformation(wTp_estimate); // Get Estimated Location
            CableRobot.CartesianError(wTp_final,wTp_estimate,Error); // obtain error between frames

            if(Error.euclideanNorm()>tol)
            {
                ROS_INFO("Do trajectory between points");
                wTp_initial=wTp_estimate;
                wTp_desired_last=wTp_estimate;
                begin=ros::Time::now();
                current_time=(ros::Time::now())-begin;

                while(ros::ok() && current_time<timefinal)
                {

                    // Get current joint position
                    CableRobot.GetRobotJointState(current_joint_position);
                    CableRobot.GetEstimatedPlatformTransformation(wTp_estimate); // Get Estimated Location
                    CableRobot.UpdatePlatformTransformation(wTp_estimate);

                    current_time=(ros::Time::now())-begin;
                    CableRobot.CartesianTrajectoryInterpolation(wTp_initial,
                                                                wTp_final,
                                                                current_time,
                                                                timefinal,
                                                                wTp_desired);

                    CableRobot.CartesianError(wTp_estimate,wTp_desired,dX); // obtain error between frames

                    dX.print(std::cout,8,"dX desired= ");
                    std::cout<<"dX =["<<std::endl;
                    for (int i = 0; i < number_of_cables; ++i) {
                        std::cout<<(dX[i])<<std::endl;
                    }
                    std::cout<<"]"<<std::endl;

                    CableRobot.calculate_jacobian(J_lau);
                    J_lau.print(std::cout,8," Laumary Jacobian = ");
                    dl_jac= J_lau*dX;
                    dl_jac.print(std::cout,8,"dl by jacobian= ");

                    CableRobot.printfM(wTp_initial,"wTp_initial(t)");
                    CableRobot.printfM(wTp_estimate,"wTp_estimate(t)");
                    CableRobot.printfM(wTp_final,"wTp_final(t)");
                    CableRobot.printfM(wTp_desired,"wTp(t)");
                    // Convert to joint motor position

                    std::cout<<"dl from motor calcutaion =["<<std::endl;
                    for (int i = 0; i < number_of_cables; ++i) {
                        current_cable_length=CableRobot.calculate_cable_length(wTp_estimate); // current_cable_length
                        desired_cable_length=CableRobot.calculate_cable_length(wTp_desired); // current_cable_length
                        dq[i]=(desired_cable_length[i]-current_cable_length[i])/ratio;
                        std::cout<<(desired_cable_length[i]-current_cable_length[i])<<std::endl;
                    }
                    std::cout<<"]"<<std::endl;


                    dq=CableRobot.calculate_motor_change(wTp_desired,ratio);


                    for (int i = 0; i < number_of_cables; ++i) {
                        desired_joint_position.position[i]=current_joint_position.position[i]+dq[i];
                    }

//                    std::cout<<"desired_joint_position =["<<std::endl;
//                    for (int i = 0; i < number_of_cables; ++i) {
//                        std::cout<<desired_joint_position.position[i]<<std::endl;
//                    }
//                    std::cout<<"]"<<std::endl;

//                    std::cout<<"current_joint_position =["<<std::endl;
//                    for (int i = 0; i < number_of_cables; ++i) {
//                        std::cout<<current_joint_position.position[i]<<std::endl;
//                    }
//                    std::cout<<"]"<<std::endl;

                    std::cout<<"dq desired =["<<std::endl;
                    for (int i = 0; i < number_of_cables; ++i) {
                        std::cout<<dq[i]<<std::endl;
                    }


                    std::cout<<"]"<<std::endl;

                    wTp_desired_last=wTp_desired;

                    ROS_INFO("==================================================");
                    ROS_INFO("==================================================");

                    joint_deviation_publisher.publish(desired_joint_position);
                    r.sleep();
                    ros::spinOnce();


                }


            }

        }

        r.sleep();

    }
    return 0;
}



