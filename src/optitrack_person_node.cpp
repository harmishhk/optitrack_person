/*
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
        ROS_INFO_STREAM_NAMED(NODE_NAME, "waiting for optitrack data to be available");
        sleep(1);
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, "optitrack data: OK");

    // initialize the publisher
    std::string full_publish_topic = std::string(NODE_NAME) + "/" + publish_topic_;

    if (subs.size()  != 0)
    {
        // create publisher of type TrackedHuman
        pub = nh_.advertise<hanp_msgs::TrackedHumans>(full_publish_topic, 1);

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
    bool isError = false,result = false;

    // get all topics from master
    if (ros::master::getTopics(topics))
    {
        // filter topic list for person only topics
        if (getSubTopics(topics, topic_base))
        {
            // create subscriber for each topic
            for (auto topic : topics)
            {
                int id = -1;
                isError = false;
                char segment_[256] = "";
                std::string segment_name;
                // get id and segment_name from the topic name
                try{
                    std::string topic_name = topic.name.substr(topic.name.find("person"),topic.name.length());
                    std::cout << topic_name << std::endl;
                    sscanf(topic_name.c_str(),"person_%d_%s",&id,segment_);
                    segment_name = std::string(segment_);
                    std::cout << id;
                    std::cout << "\t" << segment_name.empty();
                    if ((id == -1) || segment_name.empty())
                        isError = true;
                }catch (...){
                    isError = true;
                }

                // ignore the topic if doesn't match the expected pattern
                if ( isError ){
                    ROS_ERROR_STREAM_NAMED(NODE_NAME, "pattern doesn't match 'person_<id>_<segment>' in topic: " << topic.name);
                    continue;
                }

                subs.push_back(nh_.subscribe<optitrack_person::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackPerson::person_callback, this, _1, id,segment_name)));

                // make map of id of last messages with empty pointers
                //raw_messages[id][segment_name] = optitrack_person::or_pose_estimator_state::ConstPtr();

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
void OptitrackPerson::person_callback(const optitrack_person::or_pose_estimator_state::ConstPtr& msg, const int id, const std::string segment_name)
{
    if (msg->pos.size() != 0)
    {
        // update the map of pointers with latest pointer
        // since the id is same everywhere this will not insert new value in the map
        raw_messages[id][segment_name] = msg;
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found segment " << segment_name << " of person " << id);
    }
    else
    {
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "segment" << segment_name << " of person " << id << " lost");
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
    // create trackedHumans message
    hanp_msgs::TrackedHumans trackedHumans;

    // loop through all messages in raw_messages map
    for (auto human : raw_messages){
        hanp_msgs::TrackedHuman person;
        person.track_id = human.first;

        // loop through all segments of current human
        for(auto segment : human.second){
            // check if the pointer is not null
            if (segment.second){
                // discard this reading if we received this person for the first time (for proper velocity calculations)
                if (last_raw_messages.count(human.first) && last_raw_messages[human.first].count(segment.first))
                {
                    // proceed only if we've received any new data about this person
                    if (segment.second->ts.sec != last_raw_messages[human.first][segment.first]->ts.sec
                            || segment.second->ts.nsec != last_raw_messages[human.first][segment.first]->ts.nsec){

                        hanp_msgs::BodySegment body_segment;

                        // put optitrack data in to person
                        body_segment.name = segment.first;
                        processDeltaTime(human.first,segment.first,segment.second);
                        registerPose(body_segment,segment.second);
                        processVelocity(body_segment,human.first, segment.first,segment.second);
                        if (lastStates.count(human.first) && lastStates[human.first].count(segment.first)){
                            processAcceleration(body_segment,human.first,segment.first,segment.second);
                            person.segments.push_back(body_segment);
                        }
                        lastStates[human.first][segment.first] = body_segment;
                    }
                }
                // save current values for future velocity calculation
                last_raw_messages[human.first][segment.first] = segment.second;
            }
        }
        trackedHumans.tracks.push_back(person);
    }
    // add the header
    trackedHumans.header.stamp = ros::Time::now();
    trackedHumans.header.frame_id = optitrack_frame_id_;

    // publish the trackedHumans message
    pub.publish(trackedHumans);

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published persons");
}

void OptitrackPerson::registerPose(hanp_msgs::BodySegment &segment, optitrack_person::or_pose_estimator_state::ConstPtr msg){
    segment.pose.pose.position.x = msg->pos[0].x;
    segment.pose.pose.position.y = msg->pos[0].y;
    segment.pose.pose.position.z = msg->pos[0].z;
    segment.pose.pose.orientation.x = msg->pos[0].qx;
    segment.pose.pose.orientation.y = msg->pos[0].qy;
    segment.pose.pose.orientation.z = msg->pos[0].qz;
    segment.pose.pose.orientation.w = msg->pos[0].qw;


    /* not needed for PR2 => // put some covariance in human positions
    for(int index = 0; index < 6; index++)
        person.pose.covariance[index * 6 + index] = 0.1;*/
}

void OptitrackPerson::processDeltaTime(int id, std::string segment_name, optitrack_person::or_pose_estimator_state::ConstPtr msg){
    dt = (ros::Time(msg->ts.sec, msg->ts.nsec) - ros::Time(last_raw_messages[id][segment_name]->ts.sec,last_raw_messages[id][segment_name]->ts.nsec)).toSec();
}

void OptitrackPerson::processVelocity(hanp_msgs::BodySegment &segment, int id, std::string segment_name, optitrack_person::or_pose_estimator_state::ConstPtr msg){
    tf::Vector3 position_diff(msg->pos[0].x - last_raw_messages[id][segment_name]->pos[0].x,
                              msg->pos[0].y - last_raw_messages[id][segment_name]->pos[0].y,
                              msg->pos[0].z - last_raw_messages[id][segment_name]->pos[0].z);

    double roll_diff, pitch_diff, yaw_diff;
    tf::Matrix3x3(tf::Quaternion(last_raw_messages[id][segment_name]->pos[0].qx,
                                 last_raw_messages[id][segment_name]->pos[0].qy,
                                 last_raw_messages[id][segment_name]->pos[0].qz,
                                 last_raw_messages[id][segment_name]->pos[0].qw)
                .inverse() * tf::Quaternion(msg->pos[0].qx,
                                            msg->pos[0].qy,
                                            msg->pos[0].qz,
                                            msg->pos[0].qw))
                .getRPY(roll_diff, pitch_diff, yaw_diff);

    segment.twist.twist.linear.x = position_diff[0] / dt;
    segment.twist.twist.linear.y = position_diff[1] / dt;
    segment.twist.twist.linear.z = position_diff[2] / dt;
    segment.twist.twist.angular.x = roll_diff / dt;
    segment.twist.twist.angular.y = pitch_diff / dt;
    segment.twist.twist.angular.z = yaw_diff / dt;
}


void OptitrackPerson::processAcceleration(hanp_msgs::BodySegment &segment, int id, std::string segment_name, optitrack_person::or_pose_estimator_state::ConstPtr msg){
    tf::Vector3 lvelocity_diff(segment.twist.twist.linear.x  - lastStates[id][segment_name].twist.twist.linear.x,
                               segment.twist.twist.linear.y  - lastStates[id][segment_name].twist.twist.linear.y,
                               segment.twist.twist.linear.z  - lastStates[id][segment_name].twist.twist.linear.z);
    tf::Vector3 avelocity_diff(segment.twist.twist.angular.x - lastStates[id][segment_name].twist.twist.angular.x,
                               segment.twist.twist.angular.y - lastStates[id][segment_name].twist.twist.angular.y,
                               segment.twist.twist.angular.z - lastStates[id][segment_name].twist.twist.angular.z);

    segment.accel.accel.linear.x = lvelocity_diff[0] / dt;
    segment.accel.accel.linear.y = lvelocity_diff[1] / dt;
    segment.accel.accel.linear.z = lvelocity_diff[2] / dt;
    segment.accel.accel.angular.x = avelocity_diff[0] / dt;
    segment.accel.accel.angular.y = avelocity_diff[1] / dt;
    segment.accel.accel.angular.z = avelocity_diff[2] / dt;
}

// handler for something to do before killing the node


void sigintHandler(int sig){
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");
    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
    exit(sig); // necessary to interrupt during optitrackPerson initialisation
}


// the main method starts a rosnode and initializes the optitrack_person class
int main(int argc, char **argv){
    // starting the optitrack_person node
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

    // look for sigint
    signal(SIGINT, sigintHandler);

    // initializing OptitrackPerson class and passing the node handle to it
    OptitrackPerson optitrackPerson(nh, subscribe_topic_base, publish_topic_name, optitrack_frame_id, publish_rate);

    //start spinning the node
    ros::spin();

    return 0;
}
