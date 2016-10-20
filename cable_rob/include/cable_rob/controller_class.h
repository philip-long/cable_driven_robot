#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "cable_rob/robot_cables_msgs.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpMatrix.h>

#include <thread>
#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H


// A controller class for the cable drive platform
//
//  This gives has a series of methods for the cable driven paralle robot
//  such as Jacobain, IGM etc.
//  In addition to this, the class subscribes to joint states and
//  keeps its own version of platform location as well as subscrbing to others

class controller_class
{

private:
    ros::NodeHandle n;
    std::vector<vpHomogeneousMatrix> get_attachment_parameters(
            std::string param_name,ros::NodeHandle n);
    void get_initial_location(std::string param_name,ros::NodeHandle n);
    int nbr;
    ros::Subscriber joint_sub,desired_transform_sub; // joint states
    sensor_msgs::JointState joint_; // Declaration of message
    bool jointStateReceived,DesiredTransformReceived,EstimatedTransformReceived;
    vpHomogeneousMatrix wTp_,wTp_desired_,wTp_estimated_;
    // wTp_ is where this instance of class believes the platform
    // to be
    // wTp_desired_ the desired location
    // wTp_estimated_ the <<official>> or best known location of
    // platform
    void JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg); // Callback to get value of joint Sensor
    void DesiredFrameCallback(const tf2_msgs::TFMessageConstPtr& msg);

    std::thread PublisherThread;
    bool publishing_platform_;
    std::string frame_name_;
    void tfPublisher();

public: 

    controller_class(
            ros::NodeHandle nh_,
            int number_of_cables,
            std::string frame_name="platform",
            bool publishing_platform=true
            );
    sensor_msgs::JointState joint; // Public message
    std::vector<vpHomogeneousMatrix> pTbi;
    // Transform of attachment points w.r.t world frame
    std::vector<vpHomogeneousMatrix> wTbi;
    // Transform of base points w.r.t to world frame
    std::vector<vpHomogeneousMatrix> wTai;
    // Transform of platform points w.r.t to corresponding base
    std::vector<vpHomogeneousMatrix> aiTbi;
    // Current Platform Location in world frame

    std::vector<double> get_trajectory_parameter(std::string param_name);

    void printfM(vpHomogeneousMatrix M, const char *intro="Matrix");
    void printVectorDouble(std::vector<double> p,const char* intro="vector");
    void printVectorDouble(vpColVector p,const char* intro);
    double get_vector_error(vpColVector p,vpColVector p1);
    double get_vector_error(std::vector<double> p,std::vector<double> p1);

    // Kinematic functions
    void calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W);
    void calculate_inv_jacobian(vpMatrix& W);
    void calculate_jacobian(vpMatrix& J);

    // reduce jacobian by excluding rows in p
    void calculate_reduced_jacobian(vpMatrix J,vpMatrix& Jred,std::vector<int> p);

    std::vector<double> calculate_cable_length(vpHomogeneousMatrix wTp);
    std::vector<double> calculate_cable_length();

    std::vector<vpTranslationVector> calculate_cable_vectors
    (vpHomogeneousMatrix wTp);
    std::vector<vpTranslationVector> calculate_cable_vectors();

    std::vector<vpTranslationVector> calculate_normalized_cable_vectors
    (vpHomogeneousMatrix wTp);
    std::vector<vpTranslationVector> calculate_normalized_cable_vectors();

    std::vector<double> calculate_motor_change(vpHomogeneousMatrix wTp_desired, double ratio);
    // ------------------------------------



    void Stop();
    // Update the pose of the platform (for this class)
    void UpdatePlatformTransformation(vpHomogeneousMatrix M);
    void UpdatePlatformTransformation(vpTranslationVector t,vpQuaternionVector Q);
    void UpdatePlatformTransformation(double x,double y,double z
                                      ,double Rx, double Ry,double Rz);

    void convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,vpColVector omega,
                                         vpColVector& quaternion_dot);
    void convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,
                                         double omega_x,
                                         double omega_y,
                                         double omega_z,
                                         vpColVector& quaternion_dot);

    void integrate_twist(vpHomogeneousMatrix& wTp,
                                       vpColVector V);


    void GetPlatformTransformation(vpHomogeneousMatrix& M);
    void GetEstimatedPlatformTransformation(vpHomogeneousMatrix& M);
    void GetDesiredPlatformTransformation(vpHomogeneousMatrix& M);

    void PublishPlatformLocation(bool flag);

    void CartesianError(vpHomogeneousMatrix T,vpHomogeneousMatrix Td,vpColVector& Error);

    void CartesianTrajectoryInterpolation(vpHomogeneousMatrix T_initial,
                                          vpHomogeneousMatrix T_final,
                                          ros::Duration t,
                                          ros::Duration t_final,
                                          vpHomogeneousMatrix& T_desired,
                                          int Interpolator=2);

    double traj_interpolator(double tf_t, int Interpolator);

    void GetRobotJointState(sensor_msgs::JointState& return_joint);
    void SetJointFlag(bool Flag); // Set jointrecieved flag
    void SetDesiredTransformFlag(bool Flag); // Set desiredflag
    void SetEstimatedTransformFlag(bool Flag); // Set Estimated flag
    void SetPlatformFrameName(std::string frame_name); // // Set the frame name, this
    // instance of class publishes
    bool GetJointFlag(); // Get the joint recieved flag

    bool GetDesiredTransformFlag(); // Get the desired flag
    bool GetEstimatedTransformFlag(); // Get the estimated platform flag
};

#endif
