#include "ros/ros.h"
#include <time.h>       /* time_t, time, ctime */
#include "vector"
#include <deque>
#include "string"
#include <sensor_msgs/JointState.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <ros/package.h>


using namespace std;

class MotorSet{
private:
    int number_of_cables_;
    bool joint_flag_;
    vector<int> motors;
    vector<double> test_speeds;
    ros::NodeHandle n_;
    ros::Subscriber joint_sub; // Desired jonint position
    int number_of_readings; // acceleration time
    double time_max_;
    sensor_msgs::JointState joint_states;
    sensor_msgs::JointState desired_state;
    sensor_msgs::JointState published_state;
    vector<bool> selected;
    std::ofstream MotorOut;
    double tol_;
    string path;
    std::thread statePublisherThread_; // thread to publish data
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<shared_ptr<ofstream>> files;
    std::map<int,std::shared_ptr<ofstream>> file_lookup;
    bool WriteData(int motor,double speed,double torque);
    bool alive_;

public:
    MotorSet(int number_of_cables,ros::NodeHandle n,string results_path="./"):
        number_of_cables_(number_of_cables),
        n_(n),
        path(results_path)
    {
        MotorOut.open(path,std::ios::out | std::ios::trunc);
        joint_sub = n_.subscribe("/joint_state", 1, &MotorSet::JointStatesCallback, this);
        joint_flag_=false;

        for (int i = 0; i < number_of_cables_; ++i) {
            std::string joint_name="q"+boost::lexical_cast<std::string>(i+1);
            published_state.name.push_back(joint_name);
        }
        published_state.position.resize(number_of_cables_);
        published_state.velocity.resize(number_of_cables_);
        published_state.effort.resize(number_of_cables_);

        desired_state.position.resize(number_of_cables_);
        desired_state.velocity.resize(number_of_cables_);
        desired_state.effort.resize(number_of_cables_);
        selected.resize(number_of_cables_);
        number_of_readings=10; // default value
        tol_=0.001;
        alive_=true;
    }
    void ParseData(int argc, char *argv[]);
    void AssignTestSpeeds(double min_value,double max_value,double speed_step);
    void StartMotors();
    void SetReadingsForMean(int a) { number_of_readings=a;}
    void SetFilePrefix(std::string path);
    void SetTimeout(double a) { time_max_=a;}
    void statePublisher();
    bool CloseFiles();

};



// A function to parse input data and return motor set

// A function to define the speeds at which we record values


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "friction_identification");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
    ros::Rate r(500);
    ros::Publisher desired_joint_state=nh.advertise<sensor_msgs::JointState>("/desired_joint_position",1);
    spinner.start();
    int number_of_cables;
    if (!(ros::param::get("number_of_cables", number_of_cables))) {
        ROS_WARN("Default to 8 cables!!");
        number_of_cables=8;
    }

    double step=0.01;
    double upper_limit;
    int buffer;

    if (!(ros::param::get("~step_size", step)) ) {
        step=0.01;
        ROS_WARN("Default step_size %f ",step);
    }
    else
    {
        cout<<"OK";
    }

    if (!(ros::param::get("~upper_limit", upper_limit)) ) {
        upper_limit=2.0;
        ROS_WARN("Default upper limit %f ",upper_limit);
    }

    if ( !(ros::param::get("~buffer", buffer))  ) {
        buffer=100;
        ROS_WARN("Default buffer %d", buffer);
    }




    // Need to include roslib ro use ros::package::getPath
    std::string results_path = ros::package::getPath("br_motor_driver");
    cout<<results_path<<endl;
    results_path+="/results/friction";

    MotorSet motors(number_of_cables,nh);
    motors.ParseData(argc,argv);

    cout<<"==========================="<<endl;
    cout<<"This step "<<step<<endl;
    cout<<"upper_limit"<<upper_limit<<endl;
    cout<<"buffer"<<buffer<<endl;
    cout<<"==========================="<<endl;

    motors.AssignTestSpeeds(step,upper_limit,step);
    motors.SetReadingsForMean(buffer);
    motors.SetFilePrefix(results_path);
    motors.SetTimeout(20.0);
    ROS_INFO("Start Motors");
    motors.StartMotors();
    motors.CloseFiles();


    //    while(ros::ok())
    //    {
    //        r.sleep();
    //        ros::spinOnce();
    //    }


}


// Class function
void MotorSet::ParseData(int argc, char *argv[])
{

    int c;
    if(argc>1)
    {
        while ((c = getopt (argc, argv, "012345678")) != -1)
        {
            if(c!='?')
            {
                int p=((char) c)-'0';
                motors.push_back(p);
            }
            else
            {
                ROS_WARN("Invalid Option");
            }

        }

        sort(motors.begin(),motors.end());
        std::vector<int>::iterator it=std::unique(motors.begin(),motors.end());
        motors.resize(std::distance(motors.begin(),it)); // unique may reduce size of vector
    }
    else
    {
        ROS_WARN("Default to all motors!!");
        for (int var = 0; var < number_of_cables_; ++var) {
            motors.push_back(var+1);
        }
    }
    ROS_INFO("Identifying friction for motors :");
    cout<<" [ ";
    for (int var = 0; var < motors.size(); ++var) {
        cout<<motors[var]<<"  ";
    }
    cout<<"]"<<endl;

    // assign boolean values
    for (int i = 0; i < number_of_cables_; ++i) {
        if((std::find(motors.begin(), motors.end(), i+1) != motors.end()))
        {
            selected[i]=true;
        }
        else{
            selected[i]=false;
        }
    }
}

void MotorSet::SetFilePrefix(std::string path)
{


    for (int i = 0; i < motors.size(); ++i) {

        // allocate memory
        std::shared_ptr<ofstream> out(new std::ofstream);
        std::string name=path+"_"+boost::lexical_cast<std::string>(motors[i]);
        cout<<name<<endl;
        out->open(name.c_str(),std::ios::out);
        cout<<"motor"<<motors[i]<<endl;
        file_lookup.insert(std::make_pair(motors[i],out));
    }
}

bool MotorSet::WriteData(int motor,double speed,double torque){
    *(file_lookup.find(motor)->second)<<speed<<", "<<torque<<endl;
    return true;
}

bool MotorSet::CloseFiles(){
    for (int i = 0; i < motors.size(); ++i) {
        ROS_INFO("Closing file for motor %d",motors[i]);
        file_lookup.find(motors[i])->second->close();
    }
    return true;
}

void MotorSet::AssignTestSpeeds(double min_value,double max_value,double speed_step){
    for (double i = min_value; i < max_value; i+=speed_step) {
        test_speeds.push_back(i);
        test_speeds.push_back(-i);
    }
    ROS_INFO("Will test at speeds (rad^{-s}) = ");
    cout<<"[";
    for (int var = 0; var < test_speeds.size(); ++var) {
        cout<<test_speeds[var]<<" ";
    }
    cout<<"]"<<endl;
}


void MotorSet::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_states=*msg; // joint is eqaul to the value pointed to by msg
    joint_flag_=true;
}

void MotorSet::StartMotors(){
    ros::Rate r(100);
    statePublisherThread_ = std::thread(&MotorSet::statePublisher, this);

    // Cycle over all Speeds
    for (int speed_i = 0; speed_i < test_speeds.size(); ++speed_i) {

        ROS_INFO("Testing speed %f",test_speeds[speed_i]);
        std::fill(desired_state.velocity.begin(),desired_state.velocity.end(),test_speeds[speed_i]);
        vector<std::deque<double>> torques(number_of_cables_);

        // We ensure the rate of change of the mean
        // Example set number of readings to 20
        // mean= is mean (11:20)
        // mean_tm1 = is mean (1:10)
        //
        vector<double> mean1(number_of_cables_); // mean for last number of readings
        vector<double> mean2(number_of_cables_); // mean for number before that
        std::vector<bool> motor_converged(number_of_cables_);

        ros::Time begin=ros::Time::now();
        bool all_converged=false;
        int cycles=0;

        double duration=0.0;
        std::fill(motor_converged.begin(),motor_converged.end(),false);

        while(ros::ok() && !all_converged)
        {
            r.sleep();
            ros::spinOnce();


            if(joint_flag_)
            {
                cycles++;
                for (int i = 0; i < number_of_cables_; ++i) {
                    // If motor is selected
                    if(selected[i] && !motor_converged[i])
                    {

                        // Initially fill up vector
                        if(cycles<number_of_readings)
                        {
                            torques[i].push_front(joint_states.effort[i]);

                        }
                        else
                        {
                            ROS_INFO("Testing motor %d speed %f",i+1,test_speeds[speed_i]);
                            cout<<"Buffer=["<<endl;
                            for (int var = 0; var < torques[i].size(); ++var) {
                                cout<<torques[i][var]<<endl;
                            }
                            cout<<"]"<<endl;
                            torques[i].pop_back(); // remove oldest value
                            torques[i].push_front(joint_states.effort[i]); // add new value at the start
                            // returns the middle iterator
                            auto middle=std::next(torques[i].begin(),torques[i].size()/2);
                            // get mean of old data begin->middle

                            mean1[i]=std::accumulate(torques[i].begin(), middle, 0.0)/(torques[i].size()/2);
                            // get mean of most recent data middle->end
                            mean2[i]=std::accumulate(middle, torques[i].end(), 0.0)/( ( torques[i].size()- (torques[i].size()/2) ) );

                            cout<<"Most recent torque ("<<i<<") ="<<torques[i][0]<<endl;
                            cout<<"Mean of recent values of torque("<<i<<") ="<<mean1[i]<<endl;
                            cout<<"Mean of older values of torque ("<<i<<") ="<<mean2[i]<<endl;
                            cout<<" Current time = "<<(ros::Time::now()-begin).toSec()<<endl;

                            if( fabs(mean1[i]-mean2[i])<tol_ )
                            {
                                ROS_INFO("Motor %d, Mean change %f within limit %f",i,fabs(mean1[i]-mean2[i]),tol_);
                                motor_converged[i]=true; // drop flag if one motor is outside tolerance
                            }
                            else
                            {
                                ROS_WARN("Motor %d, Mean change %f outside limit %f",i,fabs(mean1[i]-mean2[i]),tol_);
                            }
                        }
                    }
                    else
                    {
                        motor_converged[i]=true;
                    }
                }

                if(std::all_of(motor_converged.begin(), motor_converged.end(), [](bool v) { return v; }))
                {
                    all_converged=true;
                }

                duration=(ros::Time::now()-begin).toSec();

                if(duration>time_max_ )
                {
                    break;
                }
                ros::spinOnce();

            }
            else{
                ROS_WARN("joint state not received");
            }

            r.sleep();
            ros::spinOnce();
        } // while loop


        if(all_converged)
        {
            ROS_INFO("Converged to stable value for all active motors");
            for (int i = 0; i < number_of_cables_; ++i) {
                if(selected[i])
                {
                    ROS_INFO("Motor %d for speed %f has torque %f",i,test_speeds[speed_i],mean1[i]);
                    WriteData(i+1,test_speeds[speed_i],mean1[i]);
                }
            }
            cout<<"================================================="<<endl;
        }
        else
        {
            ROS_ERROR("Failed to stabilise after %f seconds",duration);
            for (int i = 0; i < number_of_cables_; ++i) {
                if(selected[i])
                {
                    ROS_INFO("Motor %d for speed %f has torque %f",i,test_speeds[speed_i],mean1[i]);
                    ROS_INFO("Write Data to file");
                    WriteData(i+1,test_speeds[speed_i],NAN);
                }
            }

        }

    } // for loop
    alive_=false;
    statePublisherThread_.join(); // kill publisher thread
}


// Names of joints
void MotorSet::statePublisher(){

    ros::Publisher robot_state_pub=n_.advertise<sensor_msgs::JointState>("/desired_joint_position",1);
    ros::Rate r(250);
    ROS_INFO("Starting Publisher Thread");
    while(ros::ok() && alive_)
    {
        published_state.header.stamp=ros::Time::now();

        // Only publish for active motors

        for (int i = 0; i < number_of_cables_; ++i) {
            if(!selected[i])
            {
                published_state.velocity[i]=0.0;
                published_state.effort[i]=0.0;
            }
            else
            {   // ensuring no conflict in threads
                published_state.velocity[i]=desired_state.velocity[i];
                published_state.effort[i]=desired_state.effort[i];
            }
        }
        robot_state_pub.publish(published_state);
        r.sleep();
    }
}
