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

// Node which completes the odmetry it assumes that robot is functioning perfectly and
// all constraints have been respected.


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    int number_of_cables;
    double ratio;//=0.00034906585039886593;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);
    std::string frame_name="estimated_platform_frame";
    controller_class CableRobot(nh,number_of_cables,frame_name);

    //Transform of attachment points w.r.t to platform centre in platform frame

    //double ratio=9/3.1428;  // 360/Pi/Diameter ration*dl=dq

    // Define a ration that transforms from dq (rad) to l (m)
    // dq *(180/pi)*radius of drum 20mm
    //       deg2rad    radius of drum
    // dq * (pi/180) * (0.02)
    //double ratio= (0.02)


    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    ros::Time now = ros::Time(0);

    tflistener.waitForTransform("world",frame_name,
                                now, ros::Duration(4.0));

    tflistener.lookupTransform("world",frame_name,
                               now,tfstam);

    vpTranslationVector trans(tfstam.getOrigin().getX(),
                              tfstam.getOrigin().getY(),
                              tfstam.getOrigin().getZ());
    vpQuaternionVector quat(tfstam.getRotation().getX(),
                            tfstam.getRotation().getY(),
                            tfstam.getRotation().getZ(),
                            tfstam.getRotation().getW());

    sensor_msgs::JointState current_joint_state,last_joint_state,initial_joint_state;
    last_joint_state.position.resize(number_of_cables);
    current_joint_state.position.resize(number_of_cables);
    initial_joint_state.position.resize(number_of_cables);
    std::vector<double> l(number_of_cables);
    std::vector<double> l_init(number_of_cables);
    std::vector<double> l_last(number_of_cables);
    vpHomogeneousMatrix wTp(trans,quat),wTp_funct(trans,quat);
    vpHomogeneousMatrix wTp_last(trans,quat);
    vpMatrix J(6,8); // jacobian matrix
    vpMatrix J_lau(8,6); // jacobian matrix

    vpColVector dq(number_of_cables);
    vpColVector dl(number_of_cables);
    vpColVector dl_check(number_of_cables);
    vpColVector dX(6);

    CableRobot.UpdatePlatformTransformation(wTp);
    bool InitialStep=true;
    double tol=0.01; // tolerance is in metres

    bool debug=false;

    while(!CableRobot.GetJointFlag()){
        r.sleep();
    }
    CableRobot.UpdatePlatformTransformation(wTp);
    l=CableRobot.calculate_cable_length();
    l_init=l;
    l_last=l;

    CableRobot.GetRobotJointState(current_joint_state);
    initial_joint_state=current_joint_state;
    last_joint_state=current_joint_state;

    while(ros::ok())
    {

        ROS_INFO_COND(debug,"Initializing");
        if(CableRobot.GetJointFlag())
        {
            ROS_INFO_COND(debug,"Updating platform");
            CableRobot.UpdatePlatformTransformation(wTp);
            ROS_INFO_COND(debug,"Calculate cable length");
            l=CableRobot.calculate_cable_length();
            ROS_INFO_COND(debug,"Calculate joint state");
            CableRobot.GetRobotJointState(current_joint_state);
            ROS_INFO_COND(debug,"Calculate delta q");
            // Obtain how much the joint has changed
            for (int i = 0; i < number_of_cables; ++i)
                dq[i]=current_joint_state.position[i]-last_joint_state.position[i];
            ROS_INFO_COND(debug,"Calculate delta l");
            dl=dq*ratio; // converts from rad to m
            if(dl.euclideanNorm()>tol)
                ROS_FATAL("Norm dl is large %f. Linearization may not be valid",
                          dl.euclideanNorm());
            ROS_INFO_COND(debug,"Calculate Jacobian");
            CableRobot.calculate_jacobian(J_lau);
            ROS_INFO_COND(debug,"Calculate Velocity");
            dX=(J_lau.pseudoInverse())*dl;
            ROS_INFO_COND(debug,"Integrate Velocity");
            CableRobot.integrate_twist(wTp,dX);
            wTp_last=wTp;
            last_joint_state=current_joint_state; // update last position
            l_last=l;
        }


        r.sleep();

    }
    CableRobot.Stop();
    return 0;
}



