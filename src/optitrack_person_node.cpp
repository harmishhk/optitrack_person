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

#include <signal.h>

#include <optitrack_person/optitrack_person.h>

// instantiate local variables
OptitrackPerson::OptitrackPerson(ros::NodeHandle& nh, std::string& subscribe_topic_base,
                                 std::string& publish_topic, std::string& optitrack_frame_id, int publish_rate) :
nh_(nh), subscribe_topic_base_(subscribe_topic_base),
publish_topic_(publish_topic), optitrack_frame_id_(optitrack_frame_id), publish_rate_(publish_rate)
{
    // wait if optitrack is not up
    while(!subscribeToTopics(subscribe_topic_base_))
    {
        ROS_INFO_STREAM_NAMED(NODE_NAME, "waiting for optitrack data to be availabe");
        sleep(1);
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, "optitrack data: OK");

    // initialize the publisher
    std::string full_publish_topic = std::string(NODE_NAME) + "/" + publish_topic_;

    if (subs.size()  != 0)
    {
        // create publisher of type TrackedPerson
        pub = nh_.advertise<spencer_tracking_msgs::TrackedPersons>(full_publish_topic, 1);

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

                subs.push_back(nh_.subscribe<optitrack_person::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackPerson::person_callback, this, _1, id)));

                // make map of id of last messages with empty pointers
                lastMsgs[id] = optitrack_person::or_pose_estimator_state::ConstPtr();

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
void OptitrackPerson::person_callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id)
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

    // loop through all messages in lastMsgs map
    for (auto msg : lastMsgs)
    {
        // check if the pointer is not null
        if (msg.second)
        {
            // discard this reading if we received this person for the first time
            // (for proper velocity calculations)
            if (lastToLastMsgs.count(msg.first) != 0)
            {
                // proceed only if we've received any new data about this person
                if (msg.second->ts.sec != lastToLastMsgs[msg.first]->ts.sec
                    || msg.second->ts.nsec != lastToLastMsgs[msg.first]->ts.nsec)
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

                    // put some covariance in human positions
                    for(int index = 0; index < 6; index++)
                    {
                        person.pose.covariance[index * 6 + index] = 0.1;
                    }

                    // calculate linear and angular velocities
                    tf::Vector3 position_diff(msg.second->pos[0].x - lastToLastMsgs[msg.first]->pos[0].x,
                                              msg.second->pos[0].y - lastToLastMsgs[msg.first]->pos[0].y,
                                              msg.second->pos[0].z - lastToLastMsgs[msg.first]->pos[0].z);

                    double roll_diff, pitch_diff, yaw_diff;
                    tf::Matrix3x3(tf::Quaternion(lastToLastMsgs[msg.first]->pos[0].qx,
                                                 lastToLastMsgs[msg.first]->pos[0].qy,
                                                 lastToLastMsgs[msg.first]->pos[0].qz,
                                                 lastToLastMsgs[msg.first]->pos[0].qw)
                                .inverse() * tf::Quaternion(msg.second->pos[0].qx,
                                                            msg.second->pos[0].qy,
                                                            msg.second->pos[0].qz,
                                                            msg.second->pos[0].qw))
                                .getRPY(roll_diff, pitch_diff, yaw_diff);

                    auto dt = (ros::Time(msg.second->ts.sec, msg.second->ts.nsec) -
                           ros::Time(lastToLastMsgs[msg.first]->ts.sec,
                                     lastToLastMsgs[msg.first]->ts.nsec)).toSec();
                    person.twist.twist.linear.x = position_diff[0] / dt;
                    person.twist.twist.linear.y = position_diff[1] / dt;
                    person.twist.twist.linear.z = position_diff[2] / dt;
                    person.twist.twist.angular.x = roll_diff / dt;
                    person.twist.twist.angular.y = pitch_diff / dt;
                    person.twist.twist.angular.z = yaw_diff / dt;

                    trackedPersons->tracks.push_back(person);
                }
            }
            // save current values for future velocity calculation
            lastToLastMsgs[msg.first] = msg.second;
        }
    }

    // add the header
    trackedPersons->header.stamp = ros::Time::now();
    trackedPersons->header.frame_id = optitrack_frame_id_;

    // publish the trackedPersons message
    pub.publish(*trackedPersons);

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published persons");
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
    nh.param<std::string>("topic_base", subscribe_topic_base, SUBSCRIBE_TOPIC_BASE);
    nh.param<std::string>("published_topic", publish_topic_name, PUBLISH_TOPIC_NAME);
    nh.param<std::string>("optitrack_frame_id", optitrack_frame_id, OPTITRACK_FRAME);
    nh.param<int>("publish_rate", publish_rate, PUBLISH_RATE);

    // initiazling OptitrackPerson class and passing the node handle to it
    OptitrackPerson optitrackPerson(nh, subscribe_topic_base, publish_topic_name, optitrack_frame_id, publish_rate);

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
