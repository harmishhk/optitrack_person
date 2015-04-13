/*/
 * Copyright (c) 2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                  Harmish Khambhaita on Sun Jan 25 2015
 */

// defining constants
#define NODE_NAME "optitrack_person"
#define SUBSCRIBE_TOPIC_BASE "/optitrack/bodies/person"
#define PUBLISH_TOPIC_NAME "tracked_persons"
#define OPTITRACK_FRAME "optitrack"
#define PUBLISH_RATE 10
#define PUBLISH_INFO false

#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <optitrack_person/OptitrackPersonConfig.h>
#include <optitrack_person/or_pose_estimator_state.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

class OptitrackPerson
{
    public:
    // constructor and destructor definitions
    OptitrackPerson(ros::NodeHandle& nh, std::string& topic_base,
                    std::string& publish_topic, std::string& optitrack_frame_id, int publish_rate, bool publish_info);
    ~OptitrackPerson();

    private:
    ros::NodeHandle nh_;                        // private node handle
    std::string subscribe_topic_base_, publish_topic_;    // strings for topic configuration
    std::string optitrack_frame_id_;

    std::vector<ros::Subscriber> subs;          // person subscribers for optitrack
    ros::Publisher pub;                         // publisher of built message
    std::map<int, std::vector<ros::Publisher>> infoPubs;

    ros::Timer publishTimer;                    // timer for periodic publishing of built message
    int publish_rate_;
    bool publish_info_;

    // helper variables
    ros::master::V_TopicInfo topics;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastHeadMsgs;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastBodyMsgs;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastToLastHeadMsgs;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastToLastBodyMsgs;

    std::map<int, geometry_msgs::Twist> lastHeadTwistMsgs;
    std::map<int, geometry_msgs::Twist> lastBodyTwistMsgs;

    uint64_t _track_id;

    // function definitions
    bool getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base);

    bool subscribeToTopics(std::string topic_base);
    void publishPersons(const ros::TimerEvent& event);
    void publishPersonsInfo(const ros::TimerEvent& event);

    void person_callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id, const bool isHead);
};
