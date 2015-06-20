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
 *                                  Harmish Khambhaita on Mon Aug 11 2014
 */

// defining constants
#define NODE_NAME "optitrack_person"
#define SUBSCRIBE_TOPIC_BASE "/optitrack/bodies/person"
#define PUBLISH_TOPIC_NAME "tracked_persons"
#define OPTITRACK_FRAME "optitrack"
#define PUBLISH_RATE 10

#include <ros/ros.h>
#include <tf/tf.h>

#include <optitrack_person/or_pose_estimator_state.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

class OptitrackPerson
{
    public:
    // constructor and destructor definitions
    OptitrackPerson(ros::NodeHandle& nh, std::string& topic_base,
                    std::string& publish_topic, std::string& optitrack_frame_id, int publish_rate);
    ~OptitrackPerson();

    private:
    ros::NodeHandle nh_;                        // private node handle
    std::string subscribe_topic_base_, publish_topic_;    // strings for topic configuration
    std::string optitrack_frame_id_;

    std::vector<ros::Subscriber> subs;          // person subscribers for optitrack
    ros::Publisher pub;                         // publisher of built message

    ros::Timer publishTimer;                    // timer for periodic publishing of built message
    int publish_rate_;

    // helper variables
    ros::master::V_TopicInfo topics;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastMsgs;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastToLastMsgs;

    uint64_t _track_id;

    // function definitions
    bool getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base);

    bool subscribeToTopics(std::string topic_base);
    void publishPersons(const ros::TimerEvent& event);

    void person_callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id);
};
