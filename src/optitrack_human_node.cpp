#include <signal.h>
#include "ros/ros.h"
#include "optitrack/or_pose_estimator_state.h"
#include "spencer_tracking_msgs/TrackedPerson.h"

using namespace std;

class Optitrack_Human
{
public:
    Optitrack_Human (ros::NodeHandle& nh, string& topic, uint64_t id) : _nh(nh), _topic(topic), _track_id(id)
    {
        // create subscriber for the optitrack topic
        sub = _nh.subscribe (topic, 100, &Optitrack_Human::callback, this);
        ROS_INFO("created subscriber for %s", topic.c_str());
        
        // create publisher of type TrackedPerson
        pub = _nh.advertise<spencer_tracking_msgs::TrackedPerson>("optitrack_human/tracked_person", 1000);
    }
    
    ~Optitrack_Human() {ROS_INFO("destroying subscriber for %s", _topic.c_str());}
    
    // callback for the optitrack body
    void callback(const optitrack::or_pose_estimator_state::ConstPtr& msg)
    {
        if (msg->pos.size() != 0)
        {
            spencer_tracking_msgs::TrackedPerson person;
            person.track_id = _track_id;
            person.pose.pose.position.x = msg->pos[0].x;
            person.pose.pose.position.y = msg->pos[0].y;
            person.pose.pose.position.z = msg->pos[0].z;
            person.pose.pose.orientation.x = msg->pos[0].qx;
            person.pose.pose.orientation.y = msg->pos[0].qy;
            person.pose.pose.orientation.z = msg->pos[0].qz;
            person.pose.pose.orientation.w = msg->pos[0].qw;
            
            pub.publish(person);
        }
        else
        {
            ROS_INFO("person %ld lost", _track_id);
        }
    }
    
protected: // class attributes
    ros::NodeHandle _nh;
    string _topic;
    ros::Subscriber sub;
    ros::Publisher pub;
    uint64_t _track_id;
};

// handler for something to do before killing the node
void sigintHandler(int sig)
{
  ROS_INFO("will now shutdown");
  
  // the default sigint handler, it calls shutdown() on node
  ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optitrack_human");
    ros::NodeHandle nh;
    ROS_DEBUG("started optitrack_human node");
    
    // get all topics under /optitrack/boides
    ros::master::V_TopicInfo vti;
    if(ros::master::getTopics(vti)) {
        for(auto ti : vti) {
            ROS_DEBUG("looking for \"/optitrack/bodies/Human\" in %s", ti.name.c_str());
            if(ti.name.find("/optitrack/bodies") != string::npos)
                // make a subscriber for the /optitrack/bodies topic
                auto optitrack_human = new Optitrack_Human(nh, ti.name, 1);
        }
    }
    
    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();
    
    return 0;
}