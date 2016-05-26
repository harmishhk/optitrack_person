You can use this ROS node to tracks differents segments of differents persons, and get all the informations in a single message.

Dependances :
    hanp_msgs available here : https://github.com/harmishhk/hanp_msgs

Utilisation:

In optitrack, you need to name your rigid bodies this way :
    person_<id>_<segment>

    where <id> is a integer value
    and <segment> one of the following value :
        "head","torso",
        "right_shoulder","right_elbow","right_wrist","right_hip","right_knee","right_ankle",
        "left_shoulder", "left_elbow", "left_wrist", "left_hip", "left_knee", "left_ankle"

Then run the node :
    rosparam publish_rate <rate>
        with rate between 10 and 100.
    rosrun optitrack_person optitrack_person_node

You can also use the launch file:
    roslaunch optitrack_person optitrack_person.launch mcast:=239.192.168.30 publish_rate:=<rate>

There is an alternate launch file if you just want to record the raw data (in separate topic):
    roslaunch optitrack_person optitrack.launch
