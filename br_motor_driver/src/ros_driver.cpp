// This driver is based loosely on the ur_modern_driver by Thomas Timm Andersen

#include "ros/ros.h"
#include <br_motor_driver/br_robot.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "br_motor_driver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
    spinner.start();

    int config=1; // if this parameter is 1, connection will restart automatically
    int reverse_port;
    int number_of_cables;

    if (!(ros::param::get("number_of_cables", number_of_cables))) {
        ROS_WARN("Default to 8 cables!!");
        number_of_cables=8;
    }

    if ((ros::param::get("~/commuinication_port", reverse_port))) {
        if((reverse_port <= 0) or (reverse_port >= 65535)) {
            ROS_WARN("Using default 50001 as port value is not valid (Not between 1 and 65534");
            reverse_port = 50001;
        }
    }
    else
    {
        ROS_WARN("No port given default to 50001" );
        reverse_port = 50001;
    }

    // Initialise the class passing node, port and number of cables
    BRrobot interface(nh,reverse_port,number_of_cables);
    interface.StartInterface(config);
    ros::waitForShutdown();
}
