// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include <visp/vpHomogeneousMatrix.h>
#include <cable_rob/controller_class.h>

// Function to load attachment points

void create_platform(vpHomogeneousMatrix bTcentroid,
                     std::string marker_namespace,
                     int marker_id,
                     u_int32_t shapeij,
                     visualization_msgs::Marker& marker,
                     double x_scale=0.05,
                     double y_scale=0.05,
                     double z_scale=0.05,
                     float r=0.3f,
                     float g=0.3f,
                     float b=0.3f,
                     float alpha=1.0,
                     std::string frame_id="world")
{

    vpTranslationVector P;
    vpQuaternionVector Q;
    bTcentroid.extract(P);
    bTcentroid.extract(Q);
    marker.header.stamp = ros::Time::now();

    marker.pose.position.x = P[0];
    marker.pose.position.y = P[1];
    marker.pose.position.z = P[2];

    marker.pose.orientation.x = Q.x();
    marker.pose.orientation.y = Q.y();
    marker.pose.orientation.z = Q.z();
    marker.pose.orientation.w = Q.w();

    marker.header.frame_id=frame_id;
    marker.header.stamp=ros::Time::now();
    marker.ns=marker_namespace;
    marker.id=marker_id;

    marker.type = shapeij;
    marker.action=visualization_msgs::Marker::ADD;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = x_scale;
    marker.scale.y = y_scale;
    marker.scale.z = z_scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = alpha;

    marker.lifetime = ros::Duration();

}

void create_link(vpTranslationVector p1,
                 vpTranslationVector p2,
                 std::string marker_namespace,
                 int marker_id,
                 visualization_msgs::Marker& marker,
                 double x_scale=0.05,
                 double y_scale=0.05,
                 float r=0.3f,
                 float g=0.3f,
                 float b=0.3f,
                 float alpha=1.0,
                 double tol=0.0001, //tolerance in float cals
                std::string frame_id="world"
        )
{


    u_int32_t shapeij=visualization_msgs::Marker::CUBE;
    vpTranslationVector wPij,wPij_norm,n1,n2;
    vpRotationMatrix wRij;
    vpQuaternionVector Qij;

    bool UnitAxis;
    wPij=p2-p1; // vector between ij in world frame
    // we want z axis to align with this vector

    wPij_norm=wPij/wPij.euclideanNorm(); // normalize vector
    //        std::cout<<"wPij_norm= "<<wPij_norm[0]<<", "<<wPij_norm[1]<<", "<<wPij_norm[2]<<std::endl;
    //        std::cout<<"wPij= "<<wPij[0]<<", "<<wPij[1]<<", "<<wPij[2]<<std::endl;

    for (int var = 0; var < 3; ++var) {
        if(fabs(wPij_norm[var])<tol)
        {
            n1[var]=1.0;
            UnitAxis=true;
            break;
        }
    }

    if(!UnitAxis)
    {
        n1[0]=-wPij_norm[1];
        n1[1]=-wPij_norm[0];
        n1[2]=0;
    }
    n2 = vpTranslationVector::cross(wPij_norm, n1);

    //        std::cout<<"n2= "<<n2[0]<<", "<<n2[1]<<", "<<n2[2]<<std::endl;
    //        std::cout<<"n1= "<<n1[0]<<", "<<n1[1]<<", "<<n1[2]<<std::endl;

    // Build rotation matrix
    for (int i = 0; i < 3; ++i) {
        wRij[i][0]=n1[i];
        wRij[i][1]=n2[i];
        wRij[i][2]=wPij_norm[i];
    }
    //        std::cout<<"Is it a roatation "<< wRij.isARotationMatrix()<<std::endl;

    Qij.buildFrom(wRij);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Position marker halfway along
    marker.pose.position.x = p1[0]+(wPij_norm[0]*wPij.euclideanNorm()*0.5);
    marker.pose.position.y = p1[1]+(wPij_norm[1]*wPij.euclideanNorm()*0.5);
    marker.pose.position.z = p1[2]+(wPij_norm[2]*wPij.euclideanNorm()*0.5);

    marker.pose.orientation.x = Qij.x();
    marker.pose.orientation.y = Qij.y();
    marker.pose.orientation.z = Qij.z();
    marker.pose.orientation.w = Qij.w();

    //

    marker.header.frame_id=frame_id;
    marker.header.stamp=ros::Time::now();
    marker.ns=marker_namespace;
    marker.id=marker_id;

    marker.type = shapeij;
    marker.action=visualization_msgs::Marker::ADD;



    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = x_scale;
    marker.scale.y = y_scale;
    marker.scale.z = wPij.euclideanNorm();

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = alpha;

    marker.lifetime = ros::Duration();
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_vis");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(10);
    int number_of_cables;

    double ratio;
    nh.getParam("number_of_cables",number_of_cables);
    nh.getParam("drum_radius",ratio);

    controller_class CableRobot(nh,number_of_cables,"~",false); // initialise class without publisher

    vpHomogeneousMatrix wTp_estimate,pTcentroid,wTcentroid,wTbi;
    std::vector<vpTranslationVector> wPai(number_of_cables),wPbi(number_of_cables);
    visualization_msgs::Marker marker12,marker34,marker56,marker70;
    visualization_msgs::Marker marker_plat;
    for (int var = 0; var < number_of_cables; ++var) {
        CableRobot.wTai[var].extract(wPai[var]);
    }



    // Publish fixed markers first
    create_link(wPai[1],wPai[2],"Robot_Structure",1,marker12,0.07f,0.07f);
    create_link(wPai[3],wPai[4],"Robot_Structure",2,marker34,0.07f,0.07f);
    create_link(wPai[5],wPai[6],"Robot_Structure",3,marker56,0.07f,0.07f);
    create_link(wPai[7],wPai[0],"Robot_Structure",4,marker70,0.07f,0.07f);




    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker12);
    r.sleep();
    ros::spinOnce();
    marker_pub.publish(marker34);
    r.sleep();
    ros::spinOnce();
    marker_pub.publish(marker56);
    r.sleep();
    ros::spinOnce();
    marker_pub.publish(marker70);
    r.sleep();
    ros::spinOnce();


    pTcentroid.buildFrom(0.164,0.164,0.068,0.0,0.0,0.0);

    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    line_list.header.frame_id="world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns="cables";
    line_list.pose.orientation.w =1;
    line_list.id = 2;
    line_list.lifetime = ros::Duration(0);


    geometry_msgs::Point p;
    while(ros::ok())
    {
        CableRobot.GetEstimatedPlatformTransformation(wTp_estimate);
        wTcentroid=wTp_estimate*pTcentroid;
        line_list.points.clear();


        for (int var = 0; var < number_of_cables; ++var) {
            wTbi=wTp_estimate*CableRobot.pTbi[var];
            wTbi.extract(wPbi[var]);
            p.x=wPai[var][0]; p.y=wPai[var][1]; p.z=wPai[var][2];
            line_list.points.push_back(p);
            p.x=wPbi[var][0]; p.y=wPbi[var][1]; p.z=wPbi[var][2];
            line_list.points.push_back(p);
        }
        create_platform(wTcentroid,"platform",0,visualization_msgs::Marker::CUBE,
                             marker_plat,0.306,0.354,0.138,0.3f,0.3f,0.3f,1.0,"world");

        marker_pub.publish(marker_plat);
        r.sleep();
        ros::spinOnce();
        marker_pub.publish(line_list);
        r.sleep();
        r.sleep();
        ros::spinOnce();

    }
    CableRobot.Stop();
    return 0;
}


/*
 *         }
        int i=0;
        create_link(wPbi[0],wPbi[2],"Platform_Structure",1,marker_plat[i]);
        i++;
        create_link(wPbi[0],wPbi[5],"Platform_Structure",2,marker_plat[i]);
        i++;
        create_link(wPbi[0],wPbi[6],"Platform_Structure",3,marker_plat[i]);
        i++;
        create_link(wPbi[4],wPbi[1],"Platform_Structure",4,marker_plat[i]);
        i++;
        create_link(wPbi[4],wPbi[6],"Platform_Structure",5,marker_plat[i]);
        i++;
        create_link(wPbi[4],wPbi[2],"Platform_Structure",6,marker_plat[i]);
        i++;
        create_link(wPbi[3],wPbi[5],"Platform_Structure",7,marker_plat[i]);
        i++;
        create_link(wPbi[3],wPbi[1],"Platform_Structure",8,marker_plat[i]);
        i++;
        create_link(wPbi[3],wPbi[6],"Platform_Structure",9,marker_plat[i]);
        i++;
        create_link(wPbi[7],wPbi[5],"Platform_Structure",10,marker_plat[i]);
        i++;
        create_link(wPbi[7],wPbi[1],"Platform_Structure",11,marker_plat[i]);
        i++;
        create_link(wPbi[7],wPbi[2],"Platform_Structure",12,marker_plat[i]);
        i++;
        ROS_INFO("Here");*/
