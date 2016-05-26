## optitrack_person

Small [ROS](wiki.ros.org) package to have [optitrack](http://www.optitrack.com/) data from differents segments of differents persons in [hanp_msgs](https://github.com/harmishhk/hanp_msgs) format

### build

- install dependencies
  - ```ros-indigo-deksop```
  - [*optitrack-genom3*](https://git.openrobots.org/projects/optitrack-genom3) module installed from [robotpkg](http://robotpkg.openrobots.org/)
- run ```catkin_make install```

### Utilisation:

In optitrack, you need to name your rigid bodies this way :    
    ``` person_<id>_<segment> ```   
    where ```<id> ``` is a integer value
    and ```<segment>``` one of the following value :
    ``` "head","torso", "right_shoulder","right_elbow","right_wrist","right_hip","right_knee","right_ankle", "left_shoulder", "left_elbow", "left_wrist", "left_hip", "left_knee", "left_ankle" ```

Then run the node:
  > rosparam publish_rate <rate> with rate between 10 and 100.
  
  > rosrun optitrack_person optitrack_person_node

You can also use the launch file:
    ```roslaunch optitrack_person optitrack_person.launch mcast:=239.192.168.30 publish_rate:=<rate>```     
    Make sure that the adress match the one in optitrack and that the diffusion mode is set to multicast.

### Raw Data :
There is an alternate launch file if you just want the separate topics for the raw data:
    ```roslaunch optitrack_person optitrack.launch```
