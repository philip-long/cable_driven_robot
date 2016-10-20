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

// The recursive method to obtain all combinations
// given by http://ideone.com/78jkV

#define SET_SIZE 8

int set[] = {0,1,2,3,4,5,6,7};
std::vector<std::vector<int> > all_combinations;
const int tuple_size = 6;

void recursive_comb(int step_val, int array_index, std::vector<int> tuple)
{
    if (step_val == 0)
    {
        all_combinations.push_back(tuple); //<==We have the final combination
        return;
    }

    for (int i = array_index; i < SET_SIZE; i++)
    {
        tuple.push_back(set[i]);
        recursive_comb(step_val - 1, i + 1, tuple);
        tuple.pop_back();
    }

    return;
}

void init_combinations()
{
    std::vector<int> tuple;
    tuple.reserve(tuple_size); //avoids needless allocations
    recursive_comb(tuple_size, 0, tuple);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    int number_of_cables;
    double ratio;//=0.00034906585039886593;

    init_combinations(); // Generate all combinations
    //    for (int i=0; i < all_combinations.size(); i++)
    //    {
    //        std::cout << "{";
    //        for (int j=0; j < tuple_size; j++)
    //        {
    //            std::cout << all_combinations[i][j] << " ";
    //        }
    //        std::cout << "}" << std::endl;
    //    }

    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);
    std::string frame_name="estimated_platform_frame";
    controller_class CableRobot(nh,number_of_cables,frame_name);

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
    std::vector<double> l_test(number_of_cables);
    std::vector<double> l_last(number_of_cables);
    vpHomogeneousMatrix wTp(trans,quat);
    vpHomogeneousMatrix wTp_last(trans,quat);
    vpHomogeneousMatrix wdiffp;
    vpMatrix W(6,8); // -inverse transpose of jacobian matrix
    vpMatrix WT(8,6); // inverse jacobian matrix
    vpMatrix J(6,8); // jacobian matrix
    vpMatrix J_lau(8,6); // jacobian matrix
    vpMatrix J_red(6,6); // jacobian matrix
    std::vector<int> red_rows(6);
    vpColVector dq(number_of_cables);
    vpColVector dl(number_of_cables);
    double error_integration;
    vpColVector dl_check(number_of_cables);
    vpColVector dl_red(6);
    vpColVector dX(6);

    CableRobot.UpdatePlatformTransformation(wTp);
    bool InitialStep=true;
    double tol=0.02; // tolerance is in metres

    bool debug=true;


    while(!CableRobot.GetJointFlag()){
        r.sleep();
    }

    CableRobot.UpdatePlatformTransformation(wTp);
    l=CableRobot.calculate_cable_length();
    l_test=l;
    l_last=l;

    CableRobot.GetRobotJointState(current_joint_state);
    initial_joint_state=current_joint_state;
    last_joint_state=current_joint_state;


    while(ros::ok())
    {
        std::setprecision(7);
        ROS_INFO_COND(debug,"Updating platform");
        CableRobot.UpdatePlatformTransformation(wTp);
        ROS_INFO_COND(debug,"Calculate cable length");
        l=CableRobot.calculate_cable_length();
        ROS_INFO_COND(debug,"Calculate joint state");
        CableRobot.GetRobotJointState(current_joint_state);
        ROS_INFO_COND(debug,"Calculate delta q");
        for (int i = 0; i < number_of_cables; ++i)
            dq[i]=current_joint_state.position[i]-last_joint_state.position[i];

        if(dq.euclideanNorm()>0.00001)
        {

            ROS_INFO_COND(debug,"Change detected");
            CableRobot.printVectorDouble(dq,"dq=");
            ROS_INFO_COND(debug,"Calculate delta l");
            dl=dq*ratio; // converts from rad to m
            dl_check=dq*ratio;
            CableRobot.printVectorDouble(dl_check,"dl= ");
            if(dl.euclideanNorm()>tol)
            {
                ROS_FATAL("Norm dl is large %f. Linearization may not be valid",
                          dl.euclideanNorm());
                return -1;
            }
            ROS_INFO_COND(debug,"Calculate Jacobian");
            CableRobot.calculate_jacobian(J_lau);

             dX=(J_lau.pseudoInverse())*dl; // find velocity vx vy vz wx wy wz
            CableRobot.printVectorDouble(dX,"Dx by 6x8 jacobian=");

            error_integration=100.0;
            // -------------------- start of for loops ---------------------//
            for (int i = 0; i < all_combinations.size(); ++i) {
                red_rows=all_combinations[i];

                CableRobot.calculate_reduced_jacobian(J_lau,J_red,red_rows);
              //  J_red.print(std::cout,6,"J_red");

                // Calculated dl reduced
                for (int var = 0; var < 6; ++var) {
                    dl_red[var]=dl[red_rows[var]];
                }

                CableRobot.printVectorDouble(dl_red,"dl_reduced= ");
             //   J_lau.print(std::cout,6,"J_lau");
              //  ROS_INFO_COND(debug,"Calculate Dx actual");
                dX=(J_red.inverseByQR())*dl_red; // find velocity vx vy vz wx wy wz

                CableRobot.integrate_twist(wTp,dX);
               // CableRobot.printfM(wTp,"wTp by integration= ");
                wTp_last=wTp;
                l_test=CableRobot.calculate_cable_length(wTp);
                for (int i = 0; i < number_of_cables; ++i)
                {
                    dl_check[i]=l_test[i]-l[i];
                }
                CableRobot.printVectorDouble(dl_check,"Dl by integration= ");

                CableRobot.printVectorDouble(dX,"Dx=");
              //  ROS_INFO_COND(debug,"Integrate Velocity");

                error_integration=CableRobot.get_vector_error(dl,dl_check);
               // CableRobot.printVectorDouble(dl,"dl by integration");

                if(error_integration<CableRobot.get_vector_error(dl,dl_check))
                {
                    error_integration=CableRobot.get_vector_error(dl,dl_check);
                }
                // -------------------- End of for loops ---------------------//

            }
            ROS_INFO("Min integration error %f",error_integration);

            last_joint_state=current_joint_state; // update last position
            l_last=l;
            ROS_INFO_COND(debug,"==================================================");
        }
        r.sleep();

    }
    CableRobot.Stop();
    return 0;
}



