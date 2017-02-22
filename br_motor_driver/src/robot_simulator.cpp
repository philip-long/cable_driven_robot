#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <tinyxml.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include "boost/lexical_cast.hpp"
#include <thread>
#include <ros/ros.h>


// This file acts as a simaultor for the cable driven robot.

std::vector<double> q{0.00,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> q_new{0.00,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> dq{0.001,0.001,0.003,-0.001,0.005,0.001,0.0012,-0.0045};
std::vector<double> tau{0.00,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

void error(const char *msg);
float RandomFloat(float a, float b);
std::string pack_message();
void unpack_message(std::string p);
void write_message(int sockfd);
void read_message(int sockfd);
void check_attribute(int attribute);



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "robot_simulator");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    if (argc < 3) {
        fprintf(stderr,"usage %s hostname port\n", argv[0]);
        exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");


    std::thread thread_read(read_message,sockfd);
    std::thread thread_write(write_message,sockfd);

    thread_read.join();
    thread_write.join();


    return 0;
}



void error(const char *msg)
{
    perror(msg);
    exit(0);
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}




std::string pack_message()
{


    std::string message;
    float floatvalue;
    TiXmlDocument doc;
    TiXmlElement* ROS_PLC=new TiXmlElement("PLC_ROS");
    TiXmlElement* Cables_xml=new TiXmlElement("Cables_FB");


    doc.LinkEndChild(ROS_PLC);
    ROS_PLC->LinkEndChild(Cables_xml);

    for (int i = 0; i < 8; ++i) {
        std::string element_name="Q"+
                boost::lexical_cast<std::string>(i+1);
        TiXmlElement* Qi=new TiXmlElement(element_name);
        TiXmlElement* Position=new TiXmlElement("Position");
        TiXmlElement* Velocity=new TiXmlElement("Vitesse");
        TiXmlElement* Torque=new TiXmlElement("Couple");


        floatvalue=q_new[i];
        Position->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        floatvalue=q_new[i]-q[i];
        Velocity->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        floatvalue=tau[i];
        Torque->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        Cables_xml->LinkEndChild(Qi);
        Qi->LinkEndChild(Position);
        Qi->LinkEndChild(Velocity);
        Qi->LinkEndChild(Torque);


        q[i]=q_new[i];
    }


    TiXmlPrinter printer;
    //doc.Print();
    doc.Accept( &printer );
    message = printer.CStr();


    return message;
}
void check_attribute(int attribute)
{
    switch (attribute){
    case TIXML_NO_ATTRIBUTE:
        std::cout<<"TIXML_NO_ATTRIBUTE"<<std::endl;
        break;
    case TIXML_WRONG_TYPE:
        std::cout<<"TIXML_WRONG_TYPE"<<std::endl;
        break;
    case TIXML_SUCCESS:
        break;
    default:
        ROS_INFO("who knows");
    }
}



void unpack_message(std::string p)
{
    TiXmlDocument doc;

    std::cout<<"====================================="<<std::endl;
    ROS_INFO("RECEIVED MESSAGE FROM DRIVER");
    doc.Print();
    std::cout<<"====================================="<<std::endl;

    const char* pTest =doc.Parse(p.c_str(), 0, TIXML_ENCODING_UTF8);
    bool bool_value;
    float float_value;

    TiXmlElement* PLC_ROS=doc.FirstChildElement("ROS_PLC");

  //
    // Looking under the PLC_ROS tag
    if(PLC_ROS)
    {

        TiXmlElement* Cables=PLC_ROS->FirstChildElement("Cables");
        // Looking under the Cable tag
        if(Cables)
        {
            // For each motor, get q qdot and tau, and control flag
            for (int i = 0; i < 8; ++i) {
                std::string element_name="Q"+
                        boost::lexical_cast<std::string>(i+1);
                TiXmlElement* Qi=Cables->FirstChildElement(element_name);
                if(Qi)
                {
                    TiXmlElement* Position_i=Qi->FirstChildElement("Position");
                    TiXmlElement* Torque_i=Qi->FirstChildElement("Couple");

                    if(Position_i)
                    {
                       // std::cout<<"Getting position"<<std::endl;
                        check_attribute(Position_i->
                                        QueryFloatAttribute("V",&float_value));

                        q_new[i]=float_value;

                    }
                    if(Torque_i)
                    {
                        check_attribute(Torque_i->
                                        QueryFloatAttribute("V",&float_value));

                        tau[i]=float_value;
                    }
                }

            }
        }

    }
}


void read_message(int sockfd)
{

    int n;
    char const * re;
    std::string msg;
    char buffer[256];
    ROS_INFO("Pre loop");


    while(ros::ok())
    {

        bzero(buffer,256);

         n = read(sockfd,buffer,1500);


        if (n < 0)
            error("ERROR reading from socket");

        std::string p(buffer);
        std::vector<std::string> strs;
        boost::algorithm::split_regex(strs, p, boost::regex("</ROS_PLC>"));

        for (int i = 0; i < strs.size()-1; ++i) {

            unpack_message(strs[i]);
        }

        usleep(10000);


    }
    close(sockfd);


}
void write_message(int sockfd)
{

    int n;
    char const * re;
    std::string msg;
    while(ros::ok())
    {
        msg="";
        msg=pack_message();
        msg=msg+"\0";
        re=msg.c_str();
        n = write(sockfd,re,strlen(re));

        if (n < 0)
            error("ERROR writing to socket");
        usleep(10000);


    }
    close(sockfd);
}

