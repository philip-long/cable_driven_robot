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
#include <math.h>

#include <fstream>
// Function to load attachment points


int main(int argc, char **argv) {
    ros::init(argc, argv, "CartesianController");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(200);
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



    std::vector<double> tau1; // Double means an array
    std::vector<double> tau2;
    std::vector<double> tau3;
    std::vector<double> tau4;
    std::vector<double> speed1;
    std::vector<double> speed2;
    std::vector<double> speed3;
    std::vector<double> speed4;
    std::vector<double> p_x;
    std::vector<double> p_y;
    std::vector<double> p_z;
    std::vector<double> A_1;
    std::vector<double> A_2;
    std::vector<double> A_3;
    std::vector<double> A_4;
    std::vector<double> traj_time;
    std::vector<double> Position_motor_1;
    std::vector<double> Position_motor_2;
    std::vector<double> Position_motor_3;
    std::vector<double> Position_motor_4;
    std::vector<double> Vel_motor_1;


    float traj_time_check;
    float radius_gain=0.002;
    float radius_vel_gain=0.02;
    //std::vector<double> Current;

    tau1=CableRobot.get_trajectory_parameter("tension1");ROS_INFO("1");
    tau2=CableRobot.get_trajectory_parameter("tension2");ROS_INFO("2");
    tau3=CableRobot.get_trajectory_parameter("tension3");ROS_INFO("3");
    tau4=CableRobot.get_trajectory_parameter("tension4");ROS_INFO("4");
    speed1=CableRobot.get_trajectory_parameter("speed1");ROS_INFO("5");
    speed2=CableRobot.get_trajectory_parameter("speed2");ROS_INFO("6");
    speed3=CableRobot.get_trajectory_parameter("speed3");ROS_INFO("7");
    speed4=CableRobot.get_trajectory_parameter("speed4");ROS_INFO("8");
//    p_x=CableRobot.get_trajectory_parameter("position_x");ROS_INFO("9");
//    p_y=CableRobot.get_trajectory_parameter("position_y");ROS_INFO("10");
//    p_z=CableRobot.get_trajectory_parameter("position_z");ROS_INFO("11");
    A_1=CableRobot.get_trajectory_parameter("A1");ROS_INFO("12");
    A_2=CableRobot.get_trajectory_parameter("A2");ROS_INFO("13");
    A_3=CableRobot.get_trajectory_parameter("A3");ROS_INFO("14");
    A_4=CableRobot.get_trajectory_parameter("A4");ROS_INFO("15");
    traj_time=CableRobot.get_trajectory_parameter("timeee");ROS_INFO("17");
    Position_motor_1=CableRobot.get_trajectory_parameter("Position_motor_1");ROS_INFO("18");
    Position_motor_2=CableRobot.get_trajectory_parameter("Position_motor_2");ROS_INFO("19");
    Position_motor_3=CableRobot.get_trajectory_parameter("Position_motor_3");ROS_INFO("20");
    Position_motor_4=CableRobot.get_trajectory_parameter("Position_motor_4");ROS_INFO("21");

    //Vel_motor_1=CableRobot.get_trajectory_parameter("Vel_motor_1");ROS_INFO("22");


    vpTranslationVector Position_cartersian;
    vpTranslationVector e1;
    vpTranslationVector e2;
    vpTranslationVector e3;
    vpTranslationVector e4;
    vpMatrix Wrench(3,4);
    vpMatrix Wrench_inv(4,3);
    vpTranslationVector Error_vel;
    vpTranslationVector Error_pos_motor;

    vpMatrix ForceBuffer(10,4);

   // vpTranslationVector f_PD;



    std::ofstream Position1; Position1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position1",std::ios::out | std::ios::trunc );
    std::ofstream Velocity1; Velocity1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity1",std::ios::out | std::ios::trunc );
    std::ofstream Torque1; Torque1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque1",std::ios::out | std::ios::trunc );

    std::ofstream Position2; Position2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position2",std::ios::out | std::ios::trunc );
    std::ofstream Velocity2; Velocity2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity2",std::ios::out | std::ios::trunc );
    std::ofstream Torque2; Torque2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque2",std::ios::out | std::ios::trunc );

    std::ofstream Position3; Position3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position3",std::ios::out | std::ios::trunc );
    std::ofstream Velocity3; Velocity3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity3",std::ios::out | std::ios::trunc );
    std::ofstream Torque3; Torque3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque3",std::ios::out | std::ios::trunc );

    std::ofstream Position4; Position4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position4",std::ios::out | std::ios::trunc );
    std::ofstream Velocity4; Velocity4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity4",std::ios::out | std::ios::trunc );
    std::ofstream Torque4; Torque4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque4",std::ios::out | std::ios::trunc );

    std::ofstream Position_d1; Position_d1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position_d1",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_d1; Velocity_d1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_d1",std::ios::out | std::ios::trunc );
    std::ofstream Torque_d1; Torque_d1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque_d1",std::ios::out | std::ios::trunc );

    std::ofstream Position_d2; Position_d2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position_d2",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_d2; Velocity_d2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_d2",std::ios::out | std::ios::trunc );
    std::ofstream Torque_d2; Torque_d2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque_d2",std::ios::out | std::ios::trunc );

    std::ofstream Position_d3; Position_d3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position_d3",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_d3; Velocity_d3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_d3",std::ios::out | std::ios::trunc );
    std::ofstream Torque_d3; Torque_d3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque_d3",std::ios::out | std::ios::trunc );

    std::ofstream Position_d4; Position_d4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Position_d4",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_d4; Velocity_d4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_d4",std::ios::out | std::ios::trunc );
    std::ofstream Torque_d4; Torque_d4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Torque_d4",std::ios::out | std::ios::trunc );

    std::ofstream Velocity_f1; Velocity_f1.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_f1",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_f2; Velocity_f2.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_f2",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_f3; Velocity_f3.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_f3",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_f4; Velocity_f4.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_f4",std::ios::out | std::ios::trunc );
    std::ofstream Velocity_1_round; Velocity_1_round.open("/home/caroca/catkin_ws/src/CDPR_driver/traj/Velocity_1_round",std::ios::out | std::ios::trunc );



    ros::Duration Current;
    double torque1,torque3,torque5,torque7;
    //float position_x,position_y,position_z;
    float speed_des_1,speed_des_2,speed_des_3,speed_des_4;
    ROS_INFO("working");

    ros::Duration(2.0).sleep();
    ROS_INFO("working");

    while(!CableRobot.GetJointFlag())
    {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ROS_INFO("working");




    // Parameters for shifting the direction of motors
    double Last_torque_1;
    double Last_torque_2;
    double Last_torque_3;
    double Last_torque_4;
    double diff_1;
    double diff_2;
    double diff_3;
    double diff_4;

    // Torque of Tau_ff inverse dynamics + friction
    double Tau_ff_1;
    double Tau_ff_2;
    double Tau_ff_3;
    double Tau_ff_4;
    double P_motor_des_1;
    double P_motor_des_2;
    double P_motor_des_3;
    double P_motor_des_4;

    // Parameters for friction
    double F_col_1=0.0;
    double F_col_2=0.0;
    double F_col_3=0.0;
    double F_col_4=0.0;
    double F_sta_1=0.0;
    double F_sta_2=0.0;
    double F_sta_3=0.0;
    double F_sta_4=0.0;
    double F_vis_1=0.0;
    double F_vis_2=0.0;
    double F_vis_3=0.0;
    double F_vis_4=0.0;
    double torque_friction_1=0.0;
    double torque_friction_2=0.0;
    double torque_friction_3=0.0;
    double torque_friction_4=0.0;
    double Exp_gain_1=0.0;
    double Exp_gain_2=0.0;
    double Exp_gain_3=0.0;
    double Exp_gain_4=0.0;

    // Derivative gain
    //double K_d_1=0.0012;
    double K_d_1=0.001;
    double K_d_2=K_d_1;
    double K_d_3=K_d_1;
    double K_d_4=K_d_1;

    double K_p_1=0.005;  // 0.012
    double K_p_2=K_p_1;
    double K_p_3=K_p_1;
    double K_p_4=K_p_1;



    int filter_count=0;
    double filter_output_1;
    double filter_output_2;
    double filter_output_3;
    double filter_output_4;

    std::vector<double> velocity_fil_1;
    std::vector<double> velocity_fil_2;
    std::vector<double> velocity_fil_3;
    std::vector<double> velocity_fil_4;
    std::vector<double> robot_vel_1_feedback;
    std::vector<double> robot_vel_2_feedback;
    std::vector<double> robot_vel_3_feedback;
    std::vector<double> robot_vel_4_feedback;

    // Butterforth filter coefficients

    //std::cout << std::setprecision(7) << std::fixed;




    // Filter coefficients estimated from matlab
    vpColVector A(4);
    // fast but a bit unsmooth filter
    A[0]= 1;
    A[1]= -2.9560186000000000;
    A[2]= 2.9129990000000;
    A[3]= -0.9569701000000;

    float B_0= 0.000001300000;
    float B_1= 0.000003900000;
    float B_2= B_1;
    float B_3= B_0;


    float v_1_motor_round;
    float v_2_motor_round;
    float v_3_motor_round;
    float v_4_motor_round;




    //std::cout << std::setprecision(7) << std::fixed;

    std::cout <<"B_0= "<<B_0<<", B_1= "<<B_1<<", B_2= "<<B_2<<", B_3= "<<B_3<<", A[0]= "<<A[0]<<", A[1]= "<<A[1]<<", A[2]= "<<A[2]<<", A[3]= "<<A[3]<< "\n";

    //ROS_INFO("A[0]=%f, A[1]=%f, A[2]=%f, A[3]=%f, B[0]=%f, B[1]=%f, B[2]=%f, B[3]=%f",A[0],A[1],A[2],A[3],B_0,B_1,B_2,B_3);

    ros::Time Start= ros::Time::now();
    while(ros::ok())
    {
        //ROS_INFO("Tau=[ %f, %f,%f,%f], Time=%f",tau1[i],tau2[i],tau3[i],tau4[i],traj_time[i]);



 //       for (int var = 0; var < 8; ++var) {



//        Position<<','<<CableRobot.joint.position[0];
//        Velocity<<','<<CableRobot.joint.velocity[0];
//        Torque<<','<<CableRobot.joint.effort[0];
 //       }

        // Write data on the different files of motor feedback
        Position1<<Current.toSec();Velocity1<<Current.toSec();Torque1<<Current.toSec();
        Position2<<Current.toSec();Velocity2<<Current.toSec();Torque2<<Current.toSec();
        Position3<<Current.toSec();Velocity3<<Current.toSec();Torque3<<Current.toSec();
        Position4<<Current.toSec();Velocity4<<Current.toSec();Torque4<<Current.toSec();
        Position1<<','<<CableRobot.joint.position[1];Velocity1<<','<<CableRobot.joint.velocity[1];Torque1<<','<<CableRobot.joint.effort[1];
        Position2<<','<<CableRobot.joint.position[3];Velocity2<<','<<CableRobot.joint.velocity[3];Torque2<<','<<CableRobot.joint.effort[3];
        Position3<<','<<CableRobot.joint.position[5];Velocity3<<','<<CableRobot.joint.velocity[5];Torque3<<','<<CableRobot.joint.effort[5];
        Position4<<','<<CableRobot.joint.position[7];Velocity4<<','<<CableRobot.joint.velocity[7];Torque4<<','<<CableRobot.joint.effort[7];
        //Position1<<',';Velocity1<<',';Torque1<<',';
        Position1<<std::endl;Velocity1<<std::endl;Torque1<<std::endl;
        //Position2<<',';Velocity2<<',';Torque2<<',';
        Position2<<std::endl;Velocity2<<std::endl;Torque2<<std::endl;
        //Position3<<',';Velocity3<<',';Torque3<<',';
        Position3<<std::endl;Velocity3<<std::endl;Torque3<<std::endl;
        //Position4<<',';Velocity4<<',';Torque4<<',';
        Position4<<std::endl;Velocity4<<std::endl;Torque4<<std::endl;



        torque1=0.0;
        torque3=0.0;
        torque5=0.0;
        torque7=0.0;


        Current=(ros::Time::now())-Start;
        for (int time_point = 0; time_point < traj_time.size(); ++time_point)
        {
            //ROS_INFO("time_point%f",time_point);
            if (Current.toSec()<traj_time[time_point])
            {

                // Trajectory time
                traj_time_check=traj_time[time_point];

                // Tensions from inverse dynamics
                torque1=tau1[time_point-1];
                torque3=tau2[time_point-1];
                torque5=tau3[time_point-1];
                torque7=tau4[time_point-1];

                if (traj_time_check < 15)
                {
                    // Desired motor speed
                    speed_des_1=-1*speed1[time_point-1];
                    speed_des_2=-1*speed2[time_point-1];
                    speed_des_3=-1*speed3[time_point-1];
                    speed_des_4=-1*speed4[time_point-1];

                    // Desired Cartesian position
                    //position_x=p_x[time_point-1];
                    //position_y=p_y[time_point-1];
                    //position_z=p_z[time_point-1];

                    // Motor position
                    P_motor_des_1=-1*Position_motor_1[time_point-1];
                    P_motor_des_2=-1*Position_motor_2[time_point-1];
                    P_motor_des_3=-1*Position_motor_3[time_point-1];
                    P_motor_des_4=-1*Position_motor_4[time_point-1];
                }
                else
                {
                    speed_des_1=-0;
                    speed_des_2=-0;
                    speed_des_3=-0;
                    speed_des_4=-0;

                    P_motor_des_1=-0;
                    P_motor_des_2=-0;
                    P_motor_des_3=-0;
                    P_motor_des_4=-0;
                }

                // Writing desired values on the file
                Position_d1<<traj_time_check;Velocity_d1<<traj_time_check;
                Position_d1<<','<<P_motor_des_1;Velocity_d1<<','<<speed_des_1;
                Position_d1<<std::endl;Velocity_d1<<std::endl;

                Position_d3<<traj_time_check;Velocity_d3<<traj_time_check;
                Position_d3<<','<<P_motor_des_3;Velocity_d3<<','<<speed_des_3;
                Position_d3<<std::endl;Velocity_d3<<std::endl;

                Position_d2<<traj_time_check;Velocity_d2<<traj_time_check;
                Position_d2<<','<<P_motor_des_2;Velocity_d2<<','<<speed_des_2;
                Position_d2<<std::endl;Velocity_d2<<std::endl;

                Position_d4<<traj_time_check;Velocity_d4<<traj_time_check;
                Position_d4<<','<<P_motor_des_4;Velocity_d4<<','<<speed_des_4;
                Position_d4<<std::endl;Velocity_d4<<std::endl;
                break;
            }
        }

/*
 * Wrench Matrix Calculation
 *

        Position_cartersian[0] = position_x;
        Position_cartersian[1] = position_y;
        Position_cartersian[2] = position_z;

        for (int i = 0; i < 3; ++i)
        {
            e1[i]=A_1[i]-Position_cartersian[i];
            e2[i]=A_2[i]-Position_cartersian[i];
            e3[i]=A_3[i]-Position_cartersian[i];
            e4[i]=A_4[i]-Position_cartersian[i];
           // ROS_INFO("e1[i]=%f, e2[i]=%f, e3[i]=%f, e4[i]=%f",e1[i],e2[i],e3[i],e4[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            Wrench[i][0]=e1[i]/e1.euclideanNorm();
            Wrench[i][1]=e2[i]/e2.euclideanNorm();
            Wrench[i][2]=e3[i]/e3.euclideanNorm();
            Wrench[i][3]=e4[i]/e4.euclideanNorm();
            //ROS_INFO("norm[1]=%f, norm[2]=%f, norm[3]=%f, norm[4]=%f",e1.euclideanNorm(),e2.euclideanNorm(),e3.euclideanNorm(),e4.euclideanNorm());
            //ROS_INFO("Wrench[i][1]=%f, Wrench[i][2]=%f, Wrench[i][3]=%f, Wrench[i][4]=%f",Wrench[i][1],Wrench[i][2],Wrench[i][3],Wrench[i][4]);
        }
            //Wrench.print(std::cout,12,"Jacobian=");

             Wrench_inv=Wrench.pseudoInverse();
             //Wrench_inv.print(std::cout,12,"Jacobian=");

*/
        //std::cout << std::setprecision(7) << std::fixed;

            // filter calculations

        if (traj_time_check<15)
            {
                if (filter_count<4)
                    {
                     //v_1_motor_round=Vel_motor_1[filter_count];
                     v_1_motor_round=CableRobot.joint.velocity[1];
                     v_1_motor_round = roundf(v_1_motor_round * 10000000) / 10000000;
                     robot_vel_1_feedback.push_back(v_1_motor_round);
                     velocity_fil_1.push_back(0);

                     v_2_motor_round=CableRobot.joint.velocity[3];
                     v_2_motor_round = roundf(v_2_motor_round * 10000000) / 10000000;
                     robot_vel_2_feedback.push_back(v_2_motor_round);
                     velocity_fil_2.push_back(0);

                     v_3_motor_round=CableRobot.joint.velocity[5];
                     v_3_motor_round = roundf(v_3_motor_round * 10000000) / 10000000;
                     robot_vel_3_feedback.push_back(v_3_motor_round);
                     velocity_fil_3.push_back(0);

                     v_4_motor_round=CableRobot.joint.velocity[7];
                     v_4_motor_round = roundf(v_1_motor_round * 10000000) / 10000000;
                     robot_vel_4_feedback.push_back(v_4_motor_round);
                     velocity_fil_4.push_back(0);

                    }
                else
                    {
                    v_1_motor_round=CableRobot.joint.velocity[1];
                    v_1_motor_round = roundf(v_1_motor_round * 10000000) / 10000000; // Rounding is very very important
                    robot_vel_1_feedback.push_back(v_1_motor_round);
                    filter_output_1 = (B_0*robot_vel_1_feedback[filter_count]) + (B_1*robot_vel_1_feedback[filter_count-1])
                                    + (B_2*robot_vel_1_feedback[filter_count-2]) + (B_3*robot_vel_1_feedback[filter_count-3])
                                    - (A[1]*velocity_fil_1[filter_count-1]) - (A[2]*velocity_fil_1[filter_count-2])
                                    - (A[3]*velocity_fil_1[filter_count-3]);
                    filter_output_1 = roundf(filter_output_1 * 10000000) / 10000000;
                    velocity_fil_1.push_back(filter_output_1);                    // Formula for calculating the filter


                    v_2_motor_round=CableRobot.joint.velocity[3];
                    v_2_motor_round = roundf(v_2_motor_round * 10000000) / 10000000;
                    robot_vel_2_feedback.push_back(v_2_motor_round);
                    filter_output_2 = (B_0*robot_vel_2_feedback[filter_count]) + (B_1*robot_vel_2_feedback[filter_count-1])
                                    + (B_2*robot_vel_2_feedback[filter_count-2]) + (B_3*robot_vel_2_feedback[filter_count-3])
                                    - (A[1]*velocity_fil_2[filter_count-1]) - (A[2]*velocity_fil_2[filter_count-2])
                                    - (A[3]*velocity_fil_2[filter_count-3]);
                    filter_output_2 = roundf(filter_output_2 * 10000000) / 10000000;
                    velocity_fil_2.push_back(filter_output_2);

                    v_3_motor_round=CableRobot.joint.velocity[5];
                    v_3_motor_round = roundf(v_3_motor_round * 10000000) / 10000000;
                    robot_vel_3_feedback.push_back(v_3_motor_round);
                    filter_output_3 = (B_0*robot_vel_3_feedback[filter_count]) + (B_1*robot_vel_3_feedback[filter_count-1])
                                    + (B_2*robot_vel_3_feedback[filter_count-2]) + (B_3*robot_vel_3_feedback[filter_count-3])
                                    - (A[1]*velocity_fil_3[filter_count-1]) - (A[2]*velocity_fil_3[filter_count-2])
                                    - (A[3]*velocity_fil_3[filter_count-3]);
                    filter_output_3 = roundf(filter_output_3 * 10000000) / 10000000;
                    velocity_fil_3.push_back(filter_output_3);


                    v_4_motor_round=CableRobot.joint.velocity[7];
                    v_4_motor_round = roundf(v_4_motor_round * 10000000) / 10000000;
                    robot_vel_4_feedback.push_back(v_4_motor_round);
                    filter_output_4 = (B_0*robot_vel_4_feedback[filter_count]) + (B_1*robot_vel_4_feedback[filter_count-1])
                                    + (B_2*robot_vel_4_feedback[filter_count-2]) + (B_3*robot_vel_4_feedback[filter_count-3])
                                    - (A[1]*velocity_fil_4[filter_count-1]) - (A[2]*velocity_fil_4[filter_count-2])
                                    - (A[3]*velocity_fil_4[filter_count-3]);
                    filter_output_4 = roundf(filter_output_4 * 10000000) / 10000000;
                    velocity_fil_4.push_back(filter_output_4);

                    }

                Velocity_f1<<Current.toSec();
                Velocity_f1<<','<<velocity_fil_1[filter_count];
                Velocity_f1<<std::endl;

                Velocity_f2<<Current.toSec();
                Velocity_f2<<','<<velocity_fil_2[filter_count];
                Velocity_f2<<std::endl;

                Velocity_f3<<Current.toSec();
                Velocity_f3<<','<<velocity_fil_3[filter_count];
                Velocity_f3<<std::endl;

                Velocity_f4<<Current.toSec();
                Velocity_f4<<','<<velocity_fil_4[filter_count];
                Velocity_f4<<std::endl;

                filter_count=filter_count+1;
            }



            // Estimated friction coefficients

             F_sta_1=0.0163+0.000+0.001;
             F_sta_2=0.0163+0.000+0.001;
             F_sta_3=0.0163+0.002+0.001;
             F_sta_4=0.0163-0.001+0.001;

             F_col_1=0.0163+0.000-0.002;
             F_col_2=0.0163+0.000-0.002;
             F_col_3=0.0163+0.000-0.002;
             F_col_4=0.0163-0.001-0.002;

             F_vis_1=0;
             F_vis_2=0;
             F_vis_3=0;
             F_vis_4=0;

             Exp_gain_1=0.3;
             Exp_gain_2=0.3;
             Exp_gain_3=0.3;
             Exp_gain_4=0.3;

/* Trajectory Start
*/
             desired_joint_position.header.stamp=ros::Time::now();
             if (traj_time_check < 15)
             {

              torque_friction_1=F_col_1 + (F_sta_1-F_col_1)*exp(-fabs(Exp_gain_1*speed_des_1));//+ F_vis_1*fabs(CableRobot.joint.velocity[0]);
              torque_friction_2=F_col_2 + (F_sta_1-F_col_2)*exp(-fabs(Exp_gain_2*speed_des_1));//+ F_vis_2*fabs(CableRobot.joint.velocity[2]);
              torque_friction_3=F_col_3 + (F_sta_1-F_col_3)*exp(-fabs(Exp_gain_3*speed_des_1));//+ F_vis_3*fabs(CableRobot.joint.velocity[4]);
              torque_friction_4=F_col_4 + (F_sta_1-F_col_4)*exp(-fabs(Exp_gain_4*speed_des_1));//+  F_vis_4*fabs(CableRobot.joint.velocity[7]);


              // Torque of uinverse dynqmics + friction
              // ------------------ Torque friction should always support tension torque thats why always addition
              Tau_ff_1=(radius_gain*torque1)+torque_friction_1;
              Tau_ff_2=(radius_gain*torque3)+torque_friction_2;
              Tau_ff_3=(radius_gain*torque5)+torque_friction_3;
              Tau_ff_4=(radius_gain*torque7)+torque_friction_4;


              // Torque compensation due to difference in velocity using filtered velocity

              if (fabs(speed_des_1) > fabs(velocity_fil_1[filter_count]))
                  {
                  Error_vel[0]=K_d_1*(speed_des_1-velocity_fil_1[filter_count]);
                  }
              else
                  {
                  Error_vel[0]=K_d_1*(velocity_fil_1[filter_count]-speed_des_1);
                  Error_vel[0]=-Error_vel[0];
                  }

              if (fabs(speed_des_2) > fabs(velocity_fil_2[filter_count]))
                  {
                  Error_vel[1]=K_d_2*(speed_des_2-velocity_fil_2[filter_count]);
                  }
              else
                  {
                  Error_vel[1]=K_d_2*(velocity_fil_2[filter_count]-speed_des_2);
                  Error_vel[1]=-Error_vel[1];
                  }

              if (fabs(speed_des_3) > fabs(velocity_fil_3[filter_count]))
                  {
                  Error_vel[2]=K_d_3*(speed_des_3-velocity_fil_3[filter_count]);
                  }
              else
                  {
                  Error_vel[2]=K_d_3*(velocity_fil_3[filter_count]-speed_des_3);
                  Error_vel[2]=-Error_vel[2];
                  }

              if (fabs(speed_des_4) > fabs(velocity_fil_4[filter_count]))
                  {
                  Error_vel[3]=K_d_4*(speed_des_4-velocity_fil_4[filter_count]);
                  }
              else
                  {
                  Error_vel[3]=K_d_4*(velocity_fil_4[filter_count]-speed_des_4);
                  Error_vel[3]=-Error_vel[3];
                  }


//              Error_vel[0]=K_d_1*(speed_des_1-CableRobot.joint.velocity[0]);
//              Error_vel[1]=K_d_2*(speed_des_2-CableRobot.joint.velocity[2]);
//              Error_vel[2]=K_d_3*(speed_des_3-CableRobot.joint.velocity[4]);
//              Error_vel[3]=K_d_4*(speed_des_4-CableRobot.joint.velocity[7]);





              // Torque compensation due to difference in motor position
              if (fabs(P_motor_des_1) > fabs(CableRobot.joint.position[1]))
                  {
                  Error_pos_motor[0] = K_p_1*(P_motor_des_1-CableRobot.joint.position[1]);
                  }
              else
                  {
                  Error_pos_motor[0] = K_p_1*(CableRobot.joint.position[1] - P_motor_des_1);
                  Error_pos_motor[0]=-Error_pos_motor[0];
                  }

              if (fabs(P_motor_des_2) > fabs(CableRobot.joint.position[3]))
                  {
                  Error_pos_motor[1] = K_p_2*(P_motor_des_2-CableRobot.joint.position[3]);
                  }
              else
                  {
                  Error_pos_motor[1] = K_p_2*(CableRobot.joint.position[3] - P_motor_des_2);
                  Error_pos_motor[1]=-Error_pos_motor[1];
                  }

              if (fabs(P_motor_des_3) > fabs(CableRobot.joint.position[5]))
                  {
                  Error_pos_motor[2] = K_p_3*(P_motor_des_3-CableRobot.joint.position[5]);
                  }
              else
                  {
                  Error_pos_motor[2] = K_p_3*(CableRobot.joint.position[5] - P_motor_des_3);
                  Error_pos_motor[2]=-Error_pos_motor[2];
                  }

              if (fabs(P_motor_des_4) > fabs(CableRobot.joint.position[7]))
                  {
                  Error_pos_motor[3] = K_p_4*(P_motor_des_4-CableRobot.joint.position[7]);
                  }
              else
                  {
                  Error_pos_motor[3] = K_p_4*(CableRobot.joint.position[7] - P_motor_des_4);
                  Error_pos_motor[3]=-Error_pos_motor[3];
                  }




              // Next ifelse depends when the motor switches its direction. They can be replaced by the checks on the desired velocity of the motor

              if (traj_time_check < 5.05)
              //if (traj_time_check < 5.56)
                  {
                  Tau_ff_1=(radius_gain*torque1)+torque_friction_1; // wined
                  Tau_ff_2=(radius_gain*torque3)+torque_friction_2; // wined
                  Tau_ff_3=(radius_gain*torque5)+torque_friction_3; // wined
                  Tau_ff_4=(radius_gain*torque7)+torque_friction_4; // wined

                  desired_joint_position.effort[1]=-Tau_ff_1 + Error_pos_motor[0] + Error_vel[0];//-Error_vel[0]-torque_friction;0.0195;
                  desired_joint_position.effort[3]=-Tau_ff_2 + Error_pos_motor[1] + Error_vel[1];//-torque_friction;0.0195;
                  desired_joint_position.effort[5]=-Tau_ff_3 + Error_pos_motor[2] + Error_vel[2];//-torque_friction;0.0195;
                  desired_joint_position.effort[7]=-Tau_ff_4 + Error_pos_motor[3] + Error_vel[3];//-torque_friction;0.0195;
                  }

              else if (traj_time_check > 5.05 && traj_time_check < 7.25)
                  {
                  Tau_ff_1=(radius_gain*torque1)-torque_friction_1; // Unwined
                  Tau_ff_2=(radius_gain*torque3)+torque_friction_2; // wined
                  Tau_ff_3=(radius_gain*torque5)+torque_friction_3; // wined
                  Tau_ff_4=(radius_gain*torque7)+torque_friction_4; // wined

                  desired_joint_position.effort[1]=-Tau_ff_1 + Error_pos_motor[0] + Error_vel[0];
                  desired_joint_position.effort[3]=-Tau_ff_2 + Error_pos_motor[1] + Error_vel[1];
                  desired_joint_position.effort[5]=-Tau_ff_3 + Error_pos_motor[2] + Error_vel[2];
                  desired_joint_position.effort[7]=-Tau_ff_4 + Error_pos_motor[3] + Error_vel[3];
                  }

              else if (traj_time_check > 7.25 && traj_time_check < 9.9 )
                  {
                  Tau_ff_1=(radius_gain*torque1)-torque_friction_1; // Unwined
                  Tau_ff_2=(radius_gain*torque3)-torque_friction_2; // Unwined
                  Tau_ff_3=(radius_gain*torque5)+torque_friction_3; // wined
                  Tau_ff_4=(radius_gain*torque7)-torque_friction_4; // Uwined

                  desired_joint_position.effort[1]=-Tau_ff_1 + Error_pos_motor[0] + Error_vel[0];
                  desired_joint_position.effort[3]=-Tau_ff_2 + Error_pos_motor[1] + Error_vel[1];
                  desired_joint_position.effort[5]=-Tau_ff_3 + Error_pos_motor[2] + Error_vel[2];
                  desired_joint_position.effort[7]=-Tau_ff_4 + Error_pos_motor[3] + Error_vel[3];
                  }

              else if (traj_time_check > 9.9 && traj_time_check < 15 )
                  {
                  Tau_ff_1=(radius_gain*torque1)-torque_friction_1; // Unwined
                  Tau_ff_2=(radius_gain*torque3)-torque_friction_2; // Unwined
                  Tau_ff_3=(radius_gain*torque5)-torque_friction_3; // Uwined
                  Tau_ff_4=(radius_gain*torque7)-torque_friction_4; // Uwined

                  desired_joint_position.effort[1]=-Tau_ff_1 + Error_pos_motor[0] + Error_vel[0];
                  desired_joint_position.effort[3]=-Tau_ff_2 + Error_pos_motor[1] + Error_vel[1];
                  desired_joint_position.effort[5]=-Tau_ff_3 + Error_pos_motor[2] + Error_vel[2];
                  desired_joint_position.effort[7]=-Tau_ff_4 + Error_pos_motor[3] + Error_vel[3];
                  }




/* Use only when needed to use the tension distribution

              Error_vel[0]=K_d_1*((CableRobot.joint.velocity[0] - speed_des_1)/radius_vel_gain);
              Error_vel[1]=K_d_2*((CableRobot.joint.velocity[2] - speed_des_2)/radius_vel_gain);
              Error_vel[2]=K_d_3*((CableRobot.joint.velocity[4] - speed_des_3)/radius_vel_gain);
              Error_vel[3]=K_d_4*((CableRobot.joint.velocity[7] - speed_des_4)/radius_vel_gain);


              // Force due to error between cale velocities
              for (int i = 0; i < 3; ++i)
              {
                  f_PD[i]= Wrench[i][0]*Error_vel[0]+ Wrench[i][0]*Error_vel[1]+ Wrench[i][0]*Error_vel[2]+ Wrench[i][0]*Error_vel[3];
              }
*/



              ROS_INFO("v_1=%f,v_1d=%f,err_v1=%f, p_1=%f,p_1d=%f,err_p1=%f, t_f1=%f, t_tot1=%f,T=%f, count=%d",
                      CableRobot.joint.velocity[1], speed_des_1, Error_vel[0], CableRobot.joint.position[1], P_motor_des_1, Error_pos_motor[0],
                      -Tau_ff_1, desired_joint_position.effort[1], traj_time_check,filter_count);



            }
              else
              {
                 desired_joint_position.effort[1]=-0.0;
                 desired_joint_position.effort[3]=-0.0;
                 desired_joint_position.effort[5]=-0.0;
                 desired_joint_position.effort[7]=-0.0;
                 ROS_INFO("Its end at time =%f",15.00);//traj_time[traj_time.size()-1])

              }

             Torque_d1<<traj_time_check;
             Torque_d1<<','<<desired_joint_position.effort[1];
             Torque_d1<<std::endl;

             Torque_d2<<traj_time_check;
             Torque_d2<<','<<desired_joint_position.effort[3];
             Torque_d2<<std::endl;

             Torque_d3<<traj_time_check;
             Torque_d3<<','<<desired_joint_position.effort[5];
             Torque_d3<<std::endl;


             Torque_d4<<traj_time_check;
             Torque_d4<<','<<desired_joint_position.effort[7];
             Torque_d4<<std::endl;


        joint_deviation_publisher.publish(desired_joint_position);
        ros::spinOnce();
        r.sleep();
    }
    Position1.close();Velocity1.close();Torque1.close();
    Position2.close();Velocity2.close();Torque2.close();
    Position3.close();Velocity3.close();Torque3.close();
    Position4.close();Velocity4.close();Torque4.close();

    Position_d1.close();Velocity_d1.close();
    Position_d2.close();Velocity_d2.close();
    Position_d3.close();Velocity_d3.close();
    Position_d4.close();Velocity_d4.close();

    Velocity_f1.close();Velocity_f2.close();Velocity_f3.close();Velocity_f4.close();
    Torque_d1.close();Torque_d2.close();Torque_d3.close();Torque_d4.close();

    //Velocity_1_round.close();
    return 0;

}



