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
 *                                  Harmish Khambhaita on Mon May 05 2014
 */

#define NODE_NAME "optitrack_person"
#define TOPIC_BASE "/optitrack/bodies/person"
#define PUBLISH_RATE 0.1
#define PUBLISH_TOPIC "tracked_persons"

#include <signal.h>
#include <string>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <optitrack_person/OptitrackPersonConfig.h>
#include <optitrack_person/or_pose_estimator_state.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

// #include "geometry_msgs/PointStamped.h"

class OptitrackPerson
{
public:
    OptitrackPerson(ros::NodeHandle& nh, std::string& topic_base, std::string& publish_topic) :
    nh_(nh), topic_base_(topic_base), publish_topic_(publish_topic)
    {
        // get all topics from master
        if (ros::master::getTopics(topics))
        {
            // filter topic list for person only topics
            if (getSubTopics(topics, topic_base_))
            {
                // create subscriber for each topic
                for (auto topic : topics)
                {
                    int id;
                    // get id from the topic name
                    bool isError;
                    try
                    {
                        isError = ! (std::istringstream(topic.name.substr(topic.name.size()-2, 2)) >> id);
                    }
                    catch (...)
                    {
                        isError = true;
                    }

                    // ignore the topic if no id found
                    if ( isError )
                    {
                        ROS_ERROR_STREAM_NAMED(NODE_NAME, "cannot find an id in topic: " << topic.name);
                        continue;
                    }

                    // subs.push_back(nh_.subscribe (topic.name, 1, &OptitrackPerson::callback, this));
                    subs.push_back(nh_.subscribe<optitrack_person::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackPerson::callback, this, _1, id)));

                    // make map of id of last messages with empty pointers
                    lastMsgs[id] = optitrack_person::or_pose_estimator_state::ConstPtr();

                    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "created subscriber for topic: " << topic.name);
                }
            }
            else
            {
                ROS_ERROR_STREAM_NAMED(NODE_NAME, "cannot filter topic list, no topics will be subscribed");
            }
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "cannot get topic list from the master");
        }

        // todo: make this reconfigurable
        std::string full_publish_topic = std::string(NODE_NAME) + "/" + publish_topic_;

        if (subs.size()  != 0)
        {
            // create publisher of type TrackedPerson
            pub = nh_.advertise<spencer_tracking_msgs::TrackedPersons>(full_publish_topic, 1);

            // create a publish timer
            publishTimer = nh_.createTimer(ros::Duration(PUBLISH_RATE), &OptitrackPerson::publishPersons, this);

            ROS_DEBUG_STREAM_NAMED(NODE_NAME, "created publisher for topic: " << full_publish_topic);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "no persons found. \"" << full_publish_topic << "\" will not be published");
        }

        // set up dynamic reconfigure server
        dynamic_reconfigure::Server<optitrack_person::OptitrackPersonConfig> *dsrv_ =
            new dynamic_reconfigure::Server<optitrack_person::OptitrackPersonConfig>(nh_);
        dynamic_reconfigure::Server<optitrack_person::OptitrackPersonConfig>::CallbackType cb =
            boost::bind(&OptitrackPerson::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    // some message while destroying the class
    ~OptitrackPerson()
    {
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "destroying OptitrackPerson class");
    }

    // callback for the optitrack body
    void callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id)
    {
        if (msg->pos.size() != 0)
        {
            // update the map of pointers with latest pointer
            // since the id is same everywhere this will not insert new value in the map
            lastMsgs[id] = msg;
            ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found person " << id);
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED(NODE_NAME, "person " << id << " lost");
        }
    }

private:  // class attributes
    ros::NodeHandle nh_;
    std::string topic_base_, publish_topic_;

    std::vector<ros::Subscriber> subs;
    ros::Publisher pub;

    ros::master::V_TopicInfo topics;

    ros::Timer publishTimer;
    std::map<int, optitrack_person::or_pose_estimator_state::ConstPtr> lastMsgs;
    uint64_t _track_id;

    // re-configure the optitrack_person parameters from callback
    void reconfigureCB(optitrack_person::OptitrackPersonConfig &config, uint32_t level)
    {
        // change timer rate
        if(publishTimer)
        {
            if(config.publish_rate != 0)
            {
                publishTimer.setPeriod(ros::Duration(1/config.publish_rate));
                publishTimer.start();
            }
            else
            {
                publishTimer.stop();
            }
        }
    }

    bool getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base)
    {
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "looking for \"" << topic_base << "\" string in all (" << topics.size() << ") topics");

        topics.erase(
            std::remove_if(
            std::begin(topics),
            std::end(topics),
            [topic_base](ros::master::TopicInfo topic) -> bool
                {
                  return (topic.name.find(topic_base) == std::string::npos);
                }
            ),
            std::end(topics));

        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found " << topics.size() << " topics with \"" << topic_base << "\"");

        return true;
    }

    void publishPersons(const ros::TimerEvent& event)
    {
        // create trackedPersons message, on the heap
        auto trackedPersons = new spencer_tracking_msgs::TrackedPersons();

        // loop through all messages in lastMsgs map
        for (auto msg : lastMsgs)
        {
            // check if the pointer is not null
            if (msg.second)
            {
                spencer_tracking_msgs::TrackedPerson person;

                // put optitrack data in to person
                person.track_id = msg.first;
                person.pose.pose.position.x = msg.second->pos[0].x;
                person.pose.pose.position.y = msg.second->pos[0].y;
                person.pose.pose.position.z = msg.second->pos[0].z;
                person.pose.pose.orientation.x = msg.second->pos[0].qx;
                person.pose.pose.orientation.y = msg.second->pos[0].qy;
                person.pose.pose.orientation.z = msg.second->pos[0].qz;
                person.pose.pose.orientation.w = msg.second->pos[0].qw;

                // other information, fixed for now
                person.is_occluded = false;
                person.detection_id = person.track_id;

                trackedPersons->tracks.push_back(person);
            }
        }

        // add the header
        trackedPersons->header.stamp = ros::Time::now();
        trackedPersons->header.frame_id = "optitrack";

        // publish the trackedPersons message
        pub.publish(*trackedPersons);

        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published persons");
    }
};

// handler for something to do before killing the node
void sigintHandler(int sig)
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");

    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv)
{
    // starting the optotrack_person node
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "started " << NODE_NAME << " node");

    // getting topic parameters
    std::string optitrack_topic, publish_topic;
    nh.param<std::string>("topic_base", optitrack_topic, TOPIC_BASE);
    nh.param<std::string>("published_topic", publish_topic, PUBLISH_TOPIC);

    // initiazling OptitrackPerson class and passing the node handle to it
    OptitrackPerson optitrackPerson(nh, optitrack_topic, publish_topic);

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
