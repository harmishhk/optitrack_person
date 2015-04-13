/*/
 * Copyright (c) 2015 LAAS/CNRS
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

#include <signal.h>
#include <math.h>

#include <optitrack_person/optitrack_person_full.h>

// instantiate local variables
OptitrackPerson::OptitrackPerson(ros::NodeHandle& nh, std::string& subscribe_topic_base,
    std::string& publish_topic, std::string& optitrack_frame_id, int publish_rate, bool publish_info) :
    nh_(nh), subscribe_topic_base_(subscribe_topic_base),
    publish_topic_(publish_topic), optitrack_frame_id_(optitrack_frame_id),
    publish_rate_(publish_rate), publish_info_(publish_info)
{
    // wait if optitrack is not up
    while(!subscribeToTopics(subscribe_topic_base_))
    {
        ROS_INFO_STREAM_NAMED(NODE_NAME, "waiting for optitrack data to be availabe");
        sleep(1);
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, "optitrack data: OK");

    // create the publisher topic name
    std::string full_publish_topic = std::string(NODE_NAME) + "/" + publish_topic_;

    if (subs.size()  != 0)
    {
        // create publisher of type TrackedPerson
        pub = nh_.advertise<spencer_tracking_msgs::TrackedPersons>(full_publish_topic, 1);

        // create info publishers
        if (publish_info_)
        {
            for (auto& head : lastHeadMsgs)
            {
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/x", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/y", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/z", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/roll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/pitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/yaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vx", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vy", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vz", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vroll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vpitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vyaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/vxy", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/ax", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/ay", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/az", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/aroll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/apitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/ayaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head/axy", 1));

                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/x", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/y", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/z", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/roll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/pitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/yaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vx", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vy", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vz", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vroll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vpitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vyaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/vxy", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/ax", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/ay", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/az", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/aroll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/apitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/ayaw", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/body/axy", 1));

                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/dx", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/dy", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/dz", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/droll", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/dpitch", 1));
                infoPubs[head.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(head.first) + "/head_body/dyaw", 1));
            }
            for (auto& body : lastBodyMsgs)
            {
                if ( lastHeadMsgs.find(body.first) == lastHeadMsgs.end() )
                {
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/x", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/y", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/z", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/roll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/pitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/yaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vx", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vy", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vz", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vroll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vpitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vyaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/vxy", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/ax", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/ay", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/az", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/aroll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/apitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/ayaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head/axy", 1));

                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/x", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/y", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/z", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/roll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/pitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/yaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vx", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vy", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vz", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vroll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vpitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vyaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/vxy", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/ax", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/ay", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/az", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/aroll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/apitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/ayaw", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/body/axy", 1));

                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/dx", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/dy", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/dz", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/droll", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/dpitch", 1));
                    infoPubs[body.first].push_back(nh_.advertise<std_msgs::Float64>("/person/" + std::to_string(body.first) + "/head_body/dyaw", 1));
                }
            }
        }

        // create a publish timer
        if(publish_rate_ > 0.0)
        {
            publishTimer = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
            &OptitrackPerson::publishPersons, this);
            ROS_INFO_STREAM_NAMED(NODE_NAME, "publishing: " << full_publish_topic
            << " at " << publish_rate_ << " hz");
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "publish rate cannot be < 0, nothing will be published");
        }
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "no persons found. \"" << full_publish_topic << "\" will not be published");
    }
}

// some message while destroying the class
OptitrackPerson::~OptitrackPerson()
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "destroying OptitrackPerson class");
}

bool OptitrackPerson::subscribeToTopics(std::string topic_base)
{
    bool result = false;

    // get all topics from master
    if (ros::master::getTopics(topics))
    {
        // filter topic list for person only topics
        if (getSubTopics(topics, topic_base))
        {
            // create subscriber for each topic
            for (auto topic : topics)
            {
                int id;
                bool isError;
                bool isHead;
                try
                {
                    // get id from the topic name
                    isError = ! (std::istringstream(topic.name.substr(topic.name.size()-7, 2)) >> id);

                    // get if that is head of body pose
                    isHead = (topic.name.compare(topic.name.size()-4, 4, "head") == 0);
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

                subs.push_back(nh_.subscribe<optitrack_person::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackPerson::person_callback, this, _1, id, isHead)));

                // make map of id of last messages with empty pointers
                if ( isHead )
                {
                    lastHeadMsgs[id] = optitrack_person::or_pose_estimator_state::ConstPtr();
                    lastHeadTwistMsgs[id] = geometry_msgs::Twist();
                }
                else
                {
                    lastBodyMsgs[id] = optitrack_person::or_pose_estimator_state::ConstPtr();
                    lastBodyTwistMsgs[id] = geometry_msgs::Twist();
                }

                ROS_DEBUG_STREAM_NAMED(NODE_NAME, "created subscriber for topic: " << topic.name);

                result = true;
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

    return result;
}

// callback for the optitrack body
void OptitrackPerson::person_callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id, const bool isHead)
{
    if (msg->pos.size() != 0)
    {
        // update the map of pointers with latest pointer
        // since the id is same everywhere this will not insert new value in the map
        isHead ? (lastHeadMsgs[id] = msg) : (lastBodyMsgs[id] = msg);
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found person " << id << (isHead ? " head" : " body"));
    }
    else
    {
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "person " << id << (isHead ? " head" : " body") << " lost");
    }
}


bool OptitrackPerson::getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base)
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

void OptitrackPerson::publishPersons(const ros::TimerEvent& event)
{
    // create trackedPersons message, on the heap
    auto trackedPersons = new spencer_tracking_msgs::TrackedPersons();

    // loop through all messages in lastHeadMsgs map
    for (auto msg : lastHeadMsgs)
    {
        // check if the pointer is not null
        if (msg.second)
        {
            // discard this reading if we received this person for the first time
            // (for proper velocity calculations)
            if (lastToLastHeadMsgs.count(msg.first) != 0)
            {
                // proceed only if we've received any new data about this person
                if (msg.second->ts.sec != lastToLastHeadMsgs[msg.first]->ts.sec
                    || msg.second->ts.nsec != lastToLastHeadMsgs[msg.first]->ts.nsec)
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

                    // calculate linear and angular velocities
                    auto posVec = tf::Transform(tf::Quaternion(lastToLastHeadMsgs[msg.first]->pos[0].qx,
                        lastToLastHeadMsgs[msg.first]->pos[0].qy,
                        lastToLastHeadMsgs[msg.first]->pos[0].qz,
                        lastToLastHeadMsgs[msg.first]->pos[0].qw),
                        tf::Vector3(lastToLastHeadMsgs[msg.first]->pos[0].x,
                            lastToLastHeadMsgs[msg.first]->pos[0].y,
                            lastToLastHeadMsgs[msg.first]->pos[0].z))
                            .inverse()({msg.second->pos[0].x, msg.second->pos[0].y, msg.second->pos[0].z});

                    double roll, pitch, yaw;
                    tf::Matrix3x3(tf::Quaternion(lastToLastHeadMsgs[msg.first]->pos[0].qx,
                        lastToLastHeadMsgs[msg.first]->pos[0].qy,
                        lastToLastHeadMsgs[msg.first]->pos[0].qz,
                        lastToLastHeadMsgs[msg.first]->pos[0].qw)
                        .inverse() * tf::Quaternion(msg.second->pos[0].qx,
                            msg.second->pos[0].qy,
                            msg.second->pos[0].qz,
                            msg.second->pos[0].qw))
                            .getRPY(roll, pitch, yaw);

                    auto dt = (ros::Time(msg.second->ts.sec, msg.second->ts.nsec) -
                    ros::Time(lastToLastHeadMsgs[msg.first]->ts.sec,
                        lastToLastHeadMsgs[msg.first]->ts.nsec)).toSec();
                    person.twist.twist.linear.x = posVec[0] / dt;
                    person.twist.twist.linear.y = posVec[1] / dt;
                    person.twist.twist.linear.z = posVec[2] / dt;
                    person.twist.twist.angular.x = roll / dt;
                    person.twist.twist.angular.y = pitch / dt;
                    person.twist.twist.angular.z = yaw / dt;

                    // other information, fixed for now
                    person.is_occluded = false;
                    person.detection_id = person.track_id;

                    trackedPersons->tracks.push_back(person);
                }
            }
            // save current values for future velocity calculation
            lastToLastHeadMsgs[msg.first] = msg.second;
        }
    }

    // add the header
    trackedPersons->header.stamp = ros::Time::now();
    trackedPersons->header.frame_id = optitrack_frame_id_;

    // publish the trackedPersons message
    pub.publish(*trackedPersons);

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published persons");

    if (publish_info_)
    {
        publishPersonsInfo(event);
    }
}

void OptitrackPerson::publishPersonsInfo(const ros::TimerEvent& event)
{
    std_msgs::Float64 head_x, head_y, head_z, head_roll, head_pitch, head_yaw;
    std_msgs::Float64 head_vx, head_vy, head_vz, head_vroll, head_vpitch, head_vyaw, head_vxy;
    std_msgs::Float64 head_ax, head_ay, head_az, head_aroll, head_apitch, head_ayaw, head_axy;

    std_msgs::Float64 body_x, body_y, body_z, body_roll, body_pitch, body_yaw;
    std_msgs::Float64 body_vx, body_vy, body_vz, body_vroll, body_vpitch, body_vyaw, body_vxy;
    std_msgs::Float64 body_ax, body_ay, body_az, body_aroll, body_apitch, body_ayaw, body_axy;

    std_msgs::Float64 head_body_x, head_body_y, head_body_z, head_body_roll, head_body_pitch, head_body_yaw;

    double dt_head, dt_body;

    // loop through all messages in lastHeadMsgs map
    for (auto msg : lastHeadMsgs)
    {
        // check if the pointer is not null
        if (msg.second)
        {
            // discard this reading if we received this person for the first time
            // (for proper velocity calculations)
            if (lastToLastHeadMsgs.count(msg.first) != 0)
            {
                // proceed only if we've received any new data about this person
                if (msg.second->ts.sec != lastToLastHeadMsgs[msg.first]->ts.sec
                    || msg.second->ts.nsec != lastToLastHeadMsgs[msg.first]->ts.nsec)
                {
                    head_x.data = msg.second->pos[0].x;
                    head_y.data = msg.second->pos[0].y;
                    head_z.data = msg.second->pos[0].z;
                    infoPubs[msg.first][0].publish(head_x);
                    infoPubs[msg.first][1].publish(head_y);
                    infoPubs[msg.first][2].publish(head_z);

                    tf::Matrix3x3(tf::Quaternion(msg.second->pos[0].qx,
                            msg.second->pos[0].qy,
                            msg.second->pos[0].qz,
                            msg.second->pos[0].qw))
                            .getRPY(head_roll.data, head_pitch.data, head_yaw.data);
                    infoPubs[msg.first][3].publish(head_roll);
                    infoPubs[msg.first][4].publish(head_pitch);
                    infoPubs[msg.first][5].publish(head_yaw);

                    // calculate linear and angular velocities
                    dt_head = (ros::Time(msg.second->ts.sec, msg.second->ts.nsec) -
                    ros::Time(lastToLastHeadMsgs[msg.first]->ts.sec,
                        lastToLastHeadMsgs[msg.first]->ts.nsec)).toSec();

                    head_vx.data = (msg.second->pos[0].x - lastToLastHeadMsgs[msg.first]->pos[0].x) / dt_head;
                    head_vy.data = (msg.second->pos[0].y - lastToLastHeadMsgs[msg.first]->pos[0].y) / dt_head;
                    head_vz.data = (msg.second->pos[0].z - lastToLastHeadMsgs[msg.first]->pos[0].z) / dt_head;;
                    infoPubs[msg.first][6].publish(head_vx);
                    infoPubs[msg.first][7].publish(head_vy);
                    infoPubs[msg.first][8].publish(head_vz);

                    tf::Matrix3x3(tf::Quaternion(lastToLastHeadMsgs[msg.first]->pos[0].qx,
                        lastToLastHeadMsgs[msg.first]->pos[0].qy,
                        lastToLastHeadMsgs[msg.first]->pos[0].qz,
                        lastToLastHeadMsgs[msg.first]->pos[0].qw)
                        .inverse() * tf::Quaternion(msg.second->pos[0].qx,
                            msg.second->pos[0].qy,
                            msg.second->pos[0].qz,
                            msg.second->pos[0].qw))
                            .getRPY(head_vroll.data, head_vpitch.data, head_vyaw.data);
                    head_vroll.data = head_vroll.data / dt_head;
                    head_vpitch.data = head_vpitch.data / dt_head;
                    head_vyaw.data = head_vyaw.data / dt_head;
                    infoPubs[msg.first][9].publish(head_vroll);
                    infoPubs[msg.first][10].publish(head_vpitch);
                    infoPubs[msg.first][11].publish(head_vyaw);

                    // calculate body 2d velocity (should be in the direction of motion)
                    head_vxy.data = sqrt((head_vx.data * head_vx.data) + (head_vy.data * head_vy.data));
                    infoPubs[msg.first][12].publish(head_vxy);

                    if (lastHeadTwistMsgs.count(msg.first) != 0)
                    {
                        head_ax.data = (head_vx.data - lastHeadTwistMsgs[msg.first].linear.x) / dt_head;
                        head_ay.data = (head_vy.data - lastHeadTwistMsgs[msg.first].linear.y) / dt_head;
                        head_az.data = (head_vz.data - lastHeadTwistMsgs[msg.first].linear.z) / dt_head;
                        head_aroll.data = (head_vroll.data - lastHeadTwistMsgs[msg.first].angular.x) / dt_head;
                        head_apitch.data = (head_vpitch.data - lastHeadTwistMsgs[msg.first].angular.y) / dt_head;
                        head_ayaw.data = (head_vyaw.data - lastHeadTwistMsgs[msg.first].angular.z) / dt_head;
                        auto last_axy = sqrt( (lastHeadTwistMsgs[msg.first].linear.x * lastHeadTwistMsgs[msg.first].linear.x)
                        + (lastHeadTwistMsgs[msg.first].linear.y * lastHeadTwistMsgs[msg.first].linear.y) );
                        head_axy.data = (head_vxy.data -last_axy) / dt_head;
                        infoPubs[msg.first][13].publish(head_ax);
                        infoPubs[msg.first][14].publish(head_ay);
                        infoPubs[msg.first][15].publish(head_az);
                        infoPubs[msg.first][16].publish(head_aroll);
                        infoPubs[msg.first][17].publish(head_apitch);
                        infoPubs[msg.first][18].publish(head_ayaw);
                        infoPubs[msg.first][19].publish(head_axy);
                    }
                    lastHeadTwistMsgs[msg.first].linear.x = head_vx.data;
                    lastHeadTwistMsgs[msg.first].linear.y = head_vy.data;
                    lastHeadTwistMsgs[msg.first].linear.z = head_vz.data;
                    lastHeadTwistMsgs[msg.first].angular.x = head_vroll.data;
                    lastHeadTwistMsgs[msg.first].angular.y = head_vpitch.data;
                    lastHeadTwistMsgs[msg.first].angular.z = head_vyaw.data;
                }
            }
            // save current values for future velocity calculation
            lastToLastHeadMsgs[msg.first] = msg.second;
        }
    }

    // loop through all messages in lastBodyMsgs map
    for (auto msg : lastBodyMsgs)
    {
        // check if the pointer is not null
        if (msg.second)
        {
            // discard this reading if we received this person for the first time
            // (for proper velocity calculations)
            if (lastToLastBodyMsgs.count(msg.first) != 0)
            {
                // proceed only if we've received any new data about this person
                if (msg.second->ts.sec != lastToLastBodyMsgs[msg.first]->ts.sec
                        || msg.second->ts.nsec != lastToLastBodyMsgs[msg.first]->ts.nsec)
                {
                    body_x.data = msg.second->pos[0].x;
                    body_y.data = msg.second->pos[0].y;
                    body_z.data = msg.second->pos[0].z;
                    infoPubs[msg.first][20].publish(body_x);
                    infoPubs[msg.first][21].publish(body_y);
                    infoPubs[msg.first][22].publish(body_z);

                    tf::Matrix3x3(tf::Quaternion(msg.second->pos[0].qx,
                        msg.second->pos[0].qy,
                        msg.second->pos[0].qz,
                        msg.second->pos[0].qw))
                        .getRPY(body_roll.data, body_pitch.data, body_yaw.data);
                    infoPubs[msg.first][23].publish(body_roll);
                    infoPubs[msg.first][24].publish(body_pitch);
                    infoPubs[msg.first][25].publish(body_yaw);

                    // calculate linear and angular velocities
                    dt_body = (ros::Time(msg.second->ts.sec, msg.second->ts.nsec) -
                    ros::Time(lastToLastBodyMsgs[msg.first]->ts.sec,
                        lastToLastBodyMsgs[msg.first]->ts.nsec)).toSec();

                    body_vx.data = (msg.second->pos[0].x - lastToLastBodyMsgs[msg.first]->pos[0].x) / dt_body;
                    body_vy.data = (msg.second->pos[0].y - lastToLastBodyMsgs[msg.first]->pos[0].y) / dt_body;
                    body_vz.data = (msg.second->pos[0].z - lastToLastBodyMsgs[msg.first]->pos[0].z) / dt_body;;
                    infoPubs[msg.first][26].publish(body_vx);
                    infoPubs[msg.first][27].publish(body_vy);
                    infoPubs[msg.first][28].publish(body_vz);

                    tf::Matrix3x3(tf::Quaternion(lastToLastBodyMsgs[msg.first]->pos[0].qx,
                        lastToLastBodyMsgs[msg.first]->pos[0].qy,
                        lastToLastBodyMsgs[msg.first]->pos[0].qz,
                        lastToLastBodyMsgs[msg.first]->pos[0].qw)
                        .inverse() * tf::Quaternion(msg.second->pos[0].qx,
                            msg.second->pos[0].qy,
                            msg.second->pos[0].qz,
                            msg.second->pos[0].qw))
                            .getRPY(body_vroll.data, body_vpitch.data, body_vyaw.data);
                    body_vroll.data = body_vroll.data / dt_body;
                    body_vpitch.data = body_vpitch.data / dt_body;
                    body_vyaw.data = body_vyaw.data / dt_body;
                    infoPubs[msg.first][29].publish(body_vroll);
                    infoPubs[msg.first][30].publish(body_vpitch);
                    infoPubs[msg.first][31].publish(body_vyaw);

                    // calculate body 2d velocity (should be in the direction of motion)
                    body_vxy.data = sqrt((body_vx.data * body_vx.data) + (body_vy.data * body_vy.data));
                    infoPubs[msg.first][32].publish(body_vxy);

                    if (lastBodyTwistMsgs.count(msg.first) != 0)
                    {
                        body_ax.data = (body_vx.data - lastBodyTwistMsgs[msg.first].linear.x) / dt_body;
                        body_ay.data = (body_vy.data - lastBodyTwistMsgs[msg.first].linear.y) / dt_body;
                        body_az.data = (body_vz.data - lastBodyTwistMsgs[msg.first].linear.z) / dt_body;
                        body_aroll.data = (body_vroll.data - lastBodyTwistMsgs[msg.first].angular.x) / dt_body;
                        body_apitch.data = (body_vpitch.data - lastBodyTwistMsgs[msg.first].angular.y) / dt_body;
                        body_ayaw.data = (body_vyaw.data - lastBodyTwistMsgs[msg.first].angular.z) / dt_body;
                        auto last_axy = sqrt( (lastBodyTwistMsgs[msg.first].linear.x * lastBodyTwistMsgs[msg.first].linear.x)
                        + (lastBodyTwistMsgs[msg.first].linear.y * lastBodyTwistMsgs[msg.first].linear.y) );
                        body_axy.data = (body_vxy.data -last_axy) / dt_body;
                        infoPubs[msg.first][33].publish(body_ax);
                        infoPubs[msg.first][34].publish(body_ay);
                        infoPubs[msg.first][35].publish(body_az);
                        infoPubs[msg.first][36].publish(body_aroll);
                        infoPubs[msg.first][37].publish(body_apitch);
                        infoPubs[msg.first][38].publish(body_ayaw);
                        infoPubs[msg.first][39].publish(body_axy);
                    }
                    lastBodyTwistMsgs[msg.first].linear.x = body_vx.data;
                    lastBodyTwistMsgs[msg.first].linear.y = body_vy.data;
                    lastBodyTwistMsgs[msg.first].linear.z = body_vz.data;
                    lastBodyTwistMsgs[msg.first].angular.x = body_vroll.data;
                    lastBodyTwistMsgs[msg.first].angular.y = body_vpitch.data;
                    lastBodyTwistMsgs[msg.first].angular.z = body_vyaw.data;
                }
            }
            // save current values for future velocity calculation
            lastToLastBodyMsgs[msg.first] = msg.second;

            if ( lastHeadMsgs.find(msg.first) != lastHeadMsgs.end() )
            {
                head_body_x.data = body_x.data - head_x.data;
                head_body_y.data = body_y.data - head_y.data;
                head_body_z.data = body_z.data - head_z.data;
                infoPubs[msg.first][40].publish(head_body_x);
                infoPubs[msg.first][41].publish(head_body_y);
                infoPubs[msg.first][42].publish(head_body_z);

                tf::Matrix3x3(tf::Quaternion(lastBodyMsgs[msg.first]->pos[0].qx,
                    lastBodyMsgs[msg.first]->pos[0].qy,
                    lastBodyMsgs[msg.first]->pos[0].qz,
                    lastBodyMsgs[msg.first]->pos[0].qw)
                    .inverse() * tf::Quaternion(lastHeadMsgs[msg.first]->pos[0].qx,
                        lastHeadMsgs[msg.first]->pos[0].qy,
                        lastHeadMsgs[msg.first]->pos[0].qz,
                        lastHeadMsgs[msg.first]->pos[0].qw))
                        .getRPY(head_body_roll.data, head_body_pitch.data, head_body_yaw.data);
                infoPubs[msg.first][43].publish(head_body_roll);
                infoPubs[msg.first][44].publish(head_body_pitch);
                infoPubs[msg.first][45].publish(head_body_yaw);
            }
        }
    }

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published person infos");
}

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
    std::string subscribe_topic_base, publish_topic_name, optitrack_frame_id;
    int publish_rate;
    bool publish_info;
    nh.param<std::string>("topic_base", subscribe_topic_base, SUBSCRIBE_TOPIC_BASE);
    nh.param<std::string>("published_topic", publish_topic_name, PUBLISH_TOPIC_NAME);
    nh.param<std::string>("optitrack_frame_id", optitrack_frame_id, OPTITRACK_FRAME);
    nh.param<int>("publish_rate", publish_rate, PUBLISH_RATE);
    nh.param<bool>("publish_info", publish_info, PUBLISH_INFO);

    // initiazling OptitrackPerson class and passing the node handle to it
    OptitrackPerson optitrackPerson(nh, subscribe_topic_base, publish_topic_name, optitrack_frame_id, publish_rate, publish_info);

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
